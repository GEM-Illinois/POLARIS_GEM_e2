from typing import Optional
import sys

import tensorflow

import rospy
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from gem_lanenet.lanenet_w_line_fit import LaneNetWLineFit


class LaneNetLaneDetector:
    def __init__(self, nnet: LaneNetWLineFit,
                 pub_lane: rospy.Publisher,
                 pub_mask_img: Optional[rospy.Publisher] = None) -> None:
        self._nnet = nnet
        self._pub_lane = pub_lane
        self._pub_mask_img = pub_mask_img

        self._bridge = CvBridge()

    def img_callback(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
            mask_image, offset, curvature = self._nnet.detect(cv_image)

            if self._pub_mask_img:
                mask_image_msg = self._bridge.cv2_to_imgmsg(mask_image, 'bgr8')
                self._pub_mask_img.publish(mask_image_msg)

        except CvBridgeError as e:
            rospy.logerr(e)


def main(argv) -> None:
    # Disable eager execution before initializing any LaneNet model
    tensorflow.compat.v1.disable_eager_execution()

    rospy.init_node('lanenet_node', anonymous=True)
    config_path = rospy.get_param("~config_path")
    weights_path = rospy.get_param("~weights_path")
    use_gpu = rospy.get_param("~use_gpu", True)
    debug = rospy.get_param("~debug", True)

    nnet = LaneNetWLineFit(config_path, weights_path, use_gpu)

    rospy.sleep(0)  # Wait for first simulated time message

    pub_lane = rospy.Publisher("estimated_lane", Vector3Stamped, queue_size=1)
    if debug:
        pub_mask_image = rospy.Publisher("mask_image", Image, queue_size=1)
    else:
        pub_mask_image = None

    lanenet_detector = LaneNetLaneDetector(nnet, pub_lane, pub_mask_image)

    _ = rospy.Subscriber(
            "image_raw",
            Image, lanenet_detector.img_callback, queue_size=1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down LaneNet node")


if __name__ == '__main__':
    sys.exit(main(rospy.myargv()) or 0)
