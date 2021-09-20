#!/usr/bin/env python3
from typing import Optional, Dict, List, Tuple
import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Point
from tf2_geometry_msgs import PointStamped
import tf2_ros

HALF_LANE_W = 1.83  # 1.83 meter ~ 6 feet
UNIT_IMG_W = 61  # 1 pixel represent 0.03 meter
IMG_W_UNITS, IMG_H_UNITS = 16, 9
WARPED_IMG_W = UNIT_IMG_W * IMG_W_UNITS  # 976
WARPED_IMG_H = UNIT_IMG_W * IMG_H_UNITS  # 549


class TransformFinder:
    MARKER_POS_LIST = [
        (x, y, 0.0)
        for y in np.arange(2, -3, -1) * HALF_LANE_W
        for x in np.arange(8, 2, -1) * HALF_LANE_W
    ]

    def __init__(self, tf_buffer: tf2_ros.Buffer, pub_annotated_img: rospy.Publisher):
        self._tf_buffer = tf_buffer
        self._pub_annotated_img = pub_annotated_img
        self._cam_model = None  # type: Optional[PinholeCameraModel]
        self._cv_bridge = CvBridge()
        self._src_dst_map = dict()  # type: Dict[Tuple[int, int], Tuple[int, int]]

        rospy.logdebug("List of marker positions %s" % self.MARKER_POS_LIST)

    @property
    def src_dst_map(self) -> Dict[Tuple[int, int], Tuple[int, int]]:
        return self._src_dst_map

    def cam_info_callback(self, msg: CameraInfo) -> None:
        if self._cam_model is None:
            self._cam_model = PinholeCameraModel()
        self._cam_model.fromCameraInfo(msg)

    def img_callback(self, img_msg: Image) -> None:
        if self._cam_model is None:
            rospy.logwarn("Waiting for CameraInfo message")
            return  # Do noting

        try:
            cv_img = self._cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
            rectified_cv_img = np.zeros(cv_img.shape, cv_img.dtype)
            self._cam_model.rectifyImage(cv_img, rectified_cv_img)

            src_pts = self._get_pos_to_src_pts()
            dst_pts = self._get_pos_to_dst_pts()
            self._src_dst_map = dict(zip(src_pts, dst_pts))
            src_pts_arr = np.float32(src_pts)
            dst_pts_arr = np.float32(dst_pts)
            matrix, mask = cv2.findHomography(src_pts_arr, dst_pts_arr, cv2.RANSAC, 5.0)
            warped_cv_img = cv2.warpPerspective(rectified_cv_img, matrix, (WARPED_IMG_W, WARPED_IMG_H),
                                                flags=cv2.INTER_LINEAR)

            # Mark the positions on original and warped images
            for (u, v) in src_pts:
                cv2.circle(rectified_cv_img, (u, v), radius=5, color=(0, 255, 0), thickness=-1)
            for (u, v) in dst_pts:
                cv2.circle(warped_cv_img, (u, v), radius=5, color=(0, 255, 0), thickness=-1)

            # cropped margin to aligned the size
            # base_footprint should map from (WARPED_IMG_W/2, WARPED_IMG_H) to (src_w/2, src_h)
            src_h, src_w = rectified_cv_img.shape[0:2]
            new_w_start, new_w_stop = WARPED_IMG_W//2 - src_w//2, WARPED_IMG_W//2 + src_w//2
            new_h_start, new_h_stop = WARPED_IMG_H - src_h, WARPED_IMG_H
            cropped_cv_img = warped_cv_img[new_h_start:new_h_stop, new_w_start:new_w_stop]  # row and column

            annotated_img = np.vstack((rectified_cv_img, cropped_cv_img))
            self._pub_annotated_img.publish(self._cv_bridge.cv2_to_imgmsg(annotated_img))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
        except CvBridgeError as e:
            rospy.logerr(e)

    def _get_pos_to_src_pts(self) -> List[Tuple[int, int]]:
        ret = []
        for pos in self.MARKER_POS_LIST:
            pos_stamped = PointStamped()
            pos_stamped.header.frame_id = "world"
            pos_stamped.point = Point(*pos)
            trans_pos_stamped = self._tf_buffer.transform(pos_stamped, target_frame=self._cam_model.tf_frame)
            # The navigation frame has X pointing forward, Y left and Z up, whereas the
            # vision frame has X pointing right, Y down and Z forward; hence the need to
            # reassign axes here.
            trans_pos = (-trans_pos_stamped.point.y,
                         -trans_pos_stamped.point.z,
                         trans_pos_stamped.point.x)

            u, v = self._cam_model.project3dToPixel(trans_pos)
            ret.append((int(u), int(v)))
        return ret

    def _get_pos_to_dst_pts(self) -> List[Tuple[int, int]]:
        ret = []
        for pos in self.MARKER_POS_LIST:
            pos_stamped = PointStamped()
            pos_stamped.header.frame_id = "world"
            pos_stamped.point = Point(*pos)
            trans_pos_stamped = self._tf_buffer.transform(pos_stamped, target_frame="base_footprint")
            # The navigation frame has X pointing forward and Y left, whereas the
            # cv image use row and column
            trans_uv = np.float32([-trans_pos_stamped.point.y,
                                   -trans_pos_stamped.point.x])
            # base_footprint maps to (WARPED_IMG_W/2, WARPED_IMG_H)
            u, v = trans_uv * UNIT_IMG_W / HALF_LANE_W + np.float32([WARPED_IMG_W/2, WARPED_IMG_H])
            ret.append((int(u), int(v)))
        return ret


def main(argv):
    rospy.init_node('find_perspective_transform', anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    _ = tf2_ros.TransformListener(tf_buffer)

    pub_annotated_img = rospy.Publisher("annotated_image", Image, queue_size=1)

    finder = TransformFinder(tf_buffer, pub_annotated_img)

    rospy.Subscriber(
        "camera_info",
        CameraInfo, finder.cam_info_callback, queue_size=1)
    rospy.Subscriber(
        "image_raw",
        Image, finder.img_callback, queue_size=1)
    try:
        rate = rospy.Rate(hz=0.2)
        while not rospy.is_shutdown():
            rate.sleep()
            if finder.src_dst_map:
                rospy.loginfo("\n%s" % finder.src_dst_map)
                rospy.logdebug("Pairs:\n%s" % "\n".join(["%s -> %s" % p for p in finder.src_dst_map.items()]))
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down ROS node")


if __name__ == '__main__':
    sys.exit(main(rospy.myargv()) or 0)
