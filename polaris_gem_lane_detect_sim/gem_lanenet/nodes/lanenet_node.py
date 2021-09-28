#!/usr/bin/env python3
from typing import Optional
import sys

import cv2
import numpy as np
import tensorflow

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from gem_lane_detect_msgs.msg import SimpleLaneStamped
from gem_lanenet.lanenet_w_line_fit import LaneNetWLineFit


class LaneNetLaneDetector:
    WHEEL_BASE = 1.75  # meter
    LOOKAHEAD_X = 8.0  # meter

    def __init__(self, nnet: LaneNetWLineFit,
                 frame_id: str,
                 pub_lane: rospy.Publisher,
                 pub_annotated_img: Optional[rospy.Publisher] = None,
                 ) -> None:
        self._nnet = nnet
        self._frame_id = frame_id
        self._pub_lane = pub_lane
        self._pub_annotated_img = pub_annotated_img

        self._bridge = CvBridge()

    def img_callback(self, data: Image) -> None:
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
            center_line, annotated_image = self._nnet.detect(cv_image)

            if center_line is None:
                rospy.logwarn("Cannot infer the lane center line in the current image. Skip.")
                if self._pub_annotated_img is not None:
                    annotated_image_msg = self._bridge.cv2_to_imgmsg(annotated_image, '8UC3')
                    self._pub_annotated_img.publish(annotated_image_msg)
                return

            # calculate error w.r.t the chosen frame of reference of the vehicle (ego view).
            # NOTE coefficients are in the order of y = c[0] + c[1]*x (+ ... + c[n]*x^n)
            # where x-axis is the forward direction of the ego vehicle
            yaw_err = np.arctan(center_line.convert().coef[1])

            # Calculate the offset as the distance from the chosen frame to lane center line
            if self._frame_id in ["base_footprint", "base_link"]:
                # base_footprint or base_link is the origin (0.0, 0.0)
                y_diff = center_line(0.0) - 0.0
            elif self._frame_id == "front_wheel_axle":
                # front_wheel_axle is assumed at (WHEEL_BASE, 0.0) in meters.
                y_diff = center_line(self.WHEEL_BASE) - 0.0
            else:
                rospy.logwarn(
                    "Unsupported frame_id %s for computing cross track error. " % self._frame_id
                    + "Skipping.")
                return
            offset = y_diff * np.cos(yaw_err)

            curvature = 0.0  # Using line hence the curvature is 0

            lane_stamped_msg = SimpleLaneStamped()
            lane_stamped_msg.header.stamp = rospy.Time.now()
            lane_stamped_msg.header.frame_id = self._frame_id
            lane_stamped_msg.lane.yaw_err = yaw_err
            lane_stamped_msg.lane.offset = offset
            lane_stamped_msg.lane.curvature = curvature

            self._pub_lane.publish(lane_stamped_msg)

            if self._pub_annotated_img is not None:
                label_str = 'Heading error: %.1f deg' % np.rad2deg(yaw_err)
                annotated_image = cv2.putText(annotated_image, label_str, (30, 40), 0, 1, (255, 255, 255), 2, cv2.LINE_AA)

                label_str = 'Lane center offset from vehicle: %.1f m' % offset
                annotated_image = cv2.putText(annotated_image, label_str, (30, 70), 0, 1, (255, 255, 255), 2, cv2.LINE_AA)
                annotated_image_msg = self._bridge.cv2_to_imgmsg(annotated_image, '8UC3')
                self._pub_annotated_img.publish(annotated_image_msg)

        except CvBridgeError as e:
            rospy.logerr(e)


def main(argv) -> None:
    # Disable eager execution before initializing any LaneNet model
    tensorflow.compat.v1.disable_eager_execution()

    rospy.init_node('lanenet_node', anonymous=True)
    config_path = rospy.get_param("~config_path")
    weights_path = rospy.get_param("~weights_path")
    frame_id = rospy.get_param("~frame_id", "base_footprint")
    use_gpu = rospy.get_param("~use_gpu", True)
    debug = rospy.get_param("~debug", False)

    nnet = LaneNetWLineFit(config_path, weights_path, use_gpu, debug=debug)

    pub_lane = rospy.Publisher("estimated_lane", SimpleLaneStamped, queue_size=1)
    if debug:
        pub_annotated_image = rospy.Publisher("annotated_image", Image, queue_size=1)
        lanenet_detector = LaneNetLaneDetector(nnet, frame_id, pub_lane, pub_annotated_image)
    else:
        lanenet_detector = LaneNetLaneDetector(nnet, frame_id, pub_lane)

    _ = rospy.Subscriber(
            "image_raw",
            Image, lanenet_detector.img_callback, queue_size=1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down LaneNet node")
    finally:
        nnet.close()


if __name__ == '__main__':
    sys.exit(main(rospy.myargv()) or 0)
