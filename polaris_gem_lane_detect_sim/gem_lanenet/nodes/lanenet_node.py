#!/usr/bin/env python3
from typing import Optional
import sys

import tensorflow

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from gem_lane_detect_msgs.msg import SimpleLaneStamped
from gem_lanenet.lanenet_w_line_fit import LaneNetWLineFit


class LaneNetLaneDetector:
    def __init__(self, nnet: LaneNetWLineFit,
                 pub_lane: rospy.Publisher,
                 pub_annotated_img: Optional[rospy.Publisher] = None,
                 ) -> None:
        self._nnet = nnet
        self._pub_lane = pub_lane
        self._pub_annotated_img = pub_annotated_img

        self._bridge = CvBridge()

    def img_callback(self, data: Image):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
            yaw_err, offset, curvature, annotated_image = self._nnet.detect(cv_image)

            lane_stamped_msg = SimpleLaneStamped()
            lane_stamped_msg.header.stamp = rospy.Time.now()
            lane_stamped_msg.header.frame_id = "base_link"  # TODO
            lane_stamped_msg.lane.yaw_err = yaw_err
            lane_stamped_msg.lane.offset = offset
            lane_stamped_msg.lane.curvature = curvature

            if self._pub_annotated_img is not None:
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
    use_gpu = rospy.get_param("~use_gpu", True)
    debug = rospy.get_param("~debug", False)

    nnet = LaneNetWLineFit(config_path, weights_path, use_gpu, debug=debug)

    pub_lane = rospy.Publisher("estimated_lane", SimpleLaneStamped, queue_size=1)
    if debug:
        pub_annotated_image = rospy.Publisher("annotated_image", Image, queue_size=1)
        lanenet_detector = LaneNetLaneDetector(nnet, pub_lane, pub_annotated_image)
    else:
        lanenet_detector = LaneNetLaneDetector(nnet, pub_lane)

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
