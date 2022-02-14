#!/usr/bin/env python3

import pathlib
import time
import warnings

import numpy as np

import cv_bridge
import rospy
import rosbag
import rospkg
from sensor_msgs.msg import CompressedImage

from gem_lanenet.lanenet_w_line_fit import LaneNetWLineFit
from gem_lane_detect_msgs.msg import SimpleLane, SimpleLaneStamped

CAM_IMG_TOPIC = "front_single_camera/image_raw/compressed"
LANE_TOPIC = "estimated_lane"


class ImageToLane:
    def __init__(self, lanenet: LaneNetWLineFit):
        self._lanenet = lanenet
        self._bridge = cv_bridge.CvBridge()

        self.skip_count = 0
        self.time_usage = 0.0

    def cam_img_to_lane(self, msg: CompressedImage) -> SimpleLaneStamped:
        t_begin = time.time()
        ret = self._raw_cam_img_to_lane(msg)
        self.time_usage += (time.time() - t_begin)
        return ret

    def _raw_cam_img_to_lane(self, msg: CompressedImage) -> SimpleLaneStamped:
        """ Given camera image, run LaneNet to estimate the heading and distance """

        cv_image = self._bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        center_line, annotated_img = self._lanenet.detect(cv_image)

        if center_line is None:
            warnings.warn("Cannot infer the lane center line in the current image. Skip.")
            self.skip_count += 1
            return SimpleLaneStamped(header=msg.header,
                                     lane=SimpleLane(np.nan, np.nan, np.nan))

        # calculate error w.r.t the chosen frame of reference of the vehicle (ego view).
        # NOTE coefficients are in the order of y = c[0] + c[1]*x (+ ... + c[n]*x^n)
        # where x-axis is the forward direction of the ego vehicle
        yaw_err = np.arctan(center_line.convert().coef[1])

        # Calculate the offset as the distance from base_footprint to lane center line
        # NOTE the conversion to front wheel axle as reference is done in postprocessing
        # base_footprint or base_link is the origin (0.0, 0.0)
        y_diff = center_line(0.0) - 0.0
        offset = y_diff * np.cos(yaw_err)
        ret_msg = SimpleLaneStamped(header=msg.header,
                                    lane=SimpleLane(yaw_err, offset, 0.0))
        ret_msg.header.frame_id = "base_footprint"
        return ret_msg


def process_bag_file(in_bag: rosbag.Bag, out_bag: rosbag.Bag,
                     converter: ImageToLane,
                     sec_start=0.0, sec_stop=np.inf):
    t_start = rospy.Time.from_sec(sec_start) if np.isfinite(sec_start) else None
    t_stop = rospy.Time.from_sec(sec_stop) if np.isfinite(sec_stop) else None

    progress_stamp = t_start
    stamp_itvl = rospy.Duration(secs=5)
    for topic, msg, t in in_bag.read_messages(start_time=t_start, end_time=t_stop):
        if hasattr(msg, "header"):
            t = msg.header.stamp  # Use sender's timestamp if available

        if topic == CAM_IMG_TOPIC:
            lane_msg = converter.cam_img_to_lane(msg)
            out_bag.write(LANE_TOPIC, lane_msg, t)
        else:
            out_bag.write(topic, msg, t)

        if t > progress_stamp + stamp_itvl:
            print("Finished processing "
                  "%.1f to %.1f second" % (progress_stamp.to_sec(), (progress_stamp + stamp_itvl).to_sec()))
            progress_stamp += stamp_itvl


def main(argv) -> None:
    try:
        gem_lanenet_path = pathlib.Path(rospkg.RosPack().get_path("gem_lanenet"))
    except rospkg.ResourceNotFound:
        warnings.warn("Cannot find ROS package 'gem_lanenet'."
                      "Try configure your ROS environment by `source devel/setup.bash`")
        return

    config_path = gem_lanenet_path / "configs/conf_polaris_gem.lanenet.yaml"
    weights_path = gem_lanenet_path / "lanenet_weights/tusimple_lanenet.ckpt"

    sec_start = argv.start  # second
    sec_stop = argv.stop  # second

    in_bag_name = argv.bag_file
    in_bag_path = pathlib.Path(in_bag_name)

    out_name_modifier = ".apply_lanenet-%s-%s" % (str(sec_start).zfill(3), str(sec_stop).zfill(3))
    out_bag_name = argv.output if argv.output \
        else in_bag_path.with_name(in_bag_path.stem + out_name_modifier + in_bag_path.suffix).as_posix()

    print("Saving to new bag file at %s" % out_bag_name)

    converter = ImageToLane(
        lanenet=LaneNetWLineFit(config_path=str(config_path),
                                weights_path=str(weights_path),
                                debug=False))

    begin = time.time()

    with rosbag.Bag(in_bag_name) as in_bag,\
            rosbag.Bag(out_bag_name, 'w', compression=rosbag.Compression.BZ2) as out_bag:
        num_imgs = in_bag.get_message_count(CAM_IMG_TOPIC)
        process_bag_file(in_bag, out_bag, converter, sec_start, sec_stop)

    print("Image Process Time usage: %.2f seconds" % converter.time_usage)
    print("Total Time usage: %.2f seconds" % (time.time() - begin))
    print("Skipped %d out of %d images" % (converter.skip_count, num_imgs))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Apply LaneNet on images messages in the input ROS bag "
                                                 "and store result in the output ROS bag")
    parser.add_argument('bag_file', type=str)
    parser.add_argument(
        '-o', '--output', type=str, default="",
        help="Output ROS bag file after converting images to lane.")
    parser.add_argument(
        '--start', nargs='?', type=float, default=0.0, metavar='START',
        help="Processing only those messages with timestamp >= %(metavar)s second. (default: %(default)s)")
    parser.add_argument(
        '--stop', nargs='?', type=float, default=float('inf'), metavar='STOP',
        help="Processing only those messages with timestamp < %(metavar)s second. (default: %(default)s)")
    main(parser.parse_args())
