#!/usr/bin/env python3
import pickle
import pathlib
from sys import float_info
import time
from typing import Any, NamedTuple, Tuple
import warnings

import numpy as np
from scipy.spatial.transform import Rotation

import cv_bridge
from genpy import TVal
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
import rospkg
from rosbag import Bag

from gem_lanenet.lanenet_w_line_fit import LaneNetWLineFit

PLOT_SEP = 30.0  # meter
MAX_CTE = 2.0  # meter
WHEEL_BASE = 1.75  # meter

StampedLane = NamedTuple("StampedLane", [('stamp', float), ('heading', float), ('distance', float)])
HEADING_THRES = 10 ** -4
DISTANCE_THRES = 10 ** -4

ODOM_TOPIC = "base_footprint/odom"
CAM_IMG_TOPIC = "front_single_camera/image_raw/compressed"


def is_close(l1: Tuple[float, float], l2: Tuple[float, float]) -> bool:
    return abs(l1[0] - l2[0]) <= HEADING_THRES and abs(l1[1] - l2[1]) <= DISTANCE_THRES


def quat_to_yaw(quat: Quaternion) -> float:
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    yaw, pitch, roll = Rotation.from_quat((x, y, z, w)).as_euler("ZYX")  # Follow REP-103
    return yaw


def rostime_to_msec(rt: TVal) -> float:
    """ Convert ROS Time object to millisecond in float,
    and throw warnings when the conversion may cause precision loss"""
    # Python int is unbounded Integer. No overflow.
    t_nsec = rt.to_nsec()  # type: int
    quo, rem = divmod(t_nsec, (5 ** 6))
    MAX_PRECISE_INT = float_info.radix ** float_info.mant_dig  # type: int
    if t_nsec > MAX_PRECISE_INT or rem != 0:
        warnings.warn("ROS time %s may lose precision when converted to millisecond"
                      "in floating point." % str(rt))
    return float(quo) / (2 ** 6)


def odom_to_truth(msg: Odometry) -> Tuple[float, float]:
    yaw = quat_to_yaw(msg.pose.pose.orientation)
    true_heading = -yaw  # type: float
    d = (msg.pose.pose.position.y + PLOT_SEP/2) % PLOT_SEP - PLOT_SEP/2
    assert abs(d) < PLOT_SEP/2
    true_distance = -d
    return true_heading, true_distance


class ImageToLane:
    def __init__(self, lanenet: LaneNetWLineFit, frame_id: str = "base_footprint"):
        self._lanenet = lanenet
        self._bridge = cv_bridge.CvBridge()
        self._frame_id = frame_id

        self.skip_count = 0
        self.time_usage = 0.0

    def cam_img_to_lane(self, msg: CompressedImage) -> Tuple[float, float]:
        t_begin = time.time()
        ret = self._raw_cam_img_to_lane(msg)
        self.time_usage += (time.time() - t_begin)
        return ret

    def _raw_cam_img_to_lane(self, msg: CompressedImage) -> Tuple[float, float]:
        """ Given camera image, run LaneNet to estimate the heading and distance """

        cv_image = self._bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        center_line, annotated_img = self._lanenet.detect(cv_image)

        if center_line is None:
            warnings.warn("Cannot infer the lane center line in the current image. Skip.")
            self.skip_count += 1
            return np.nan, np.nan

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
            y_diff = center_line(WHEEL_BASE) - 0.0
        else:
            warnings.warn(
                "Unsupported frame_id %s for computing cross track error. " % self._frame_id
                + "Skipping.")
            return np.nan. np.nan
        offset = y_diff * np.cos(yaw_err)
        return yaw_err, offset


def convert_bag_messages(bag: Bag, topic_to_cb, t_start=0.0, t_stop=float("inf")):
    for topic, msg, t in bag.read_messages(topics=topic_to_cb.keys()):
        if hasattr(msg, "header"):
            t = msg.header.stamp  # Use sender's timestamp if available

        t_msec_f = rostime_to_msec(t)
        if t_msec_f < t_start:
            continue
        elif t_msec_f > t_stop:
            break

        if topic in topic_to_cb:
            lane = topic_to_cb[topic](msg)
            yield topic, StampedLane(t_msec_f, *lane)
        else:
            continue  # Ignore other topics


def process_bag_file(file_name: str, topic_to_cb, sec_start=0.0, sec_stop=float("inf")):
    # Convert to milli-seconds
    t_start = sec_start * 1000
    t_stop = sec_stop * 1000
    with Bag(file_name) as bag:
        converted_iter = convert_bag_messages(bag, topic_to_cb, t_start, t_stop)

        # Merge close enough consecutive ground truths as one ground truth
        ret_dist_trace = {key: [] for key in topic_to_cb.keys()}
        progress_stamp = t_start
        stamp_itvl = 5 * 1000.0
        for topic, stamped_lane in converted_iter:
            if topic == ODOM_TOPIC and len(ret_dist_trace[topic]) > 0:
                curr_lane = (stamped_lane.heading, stamped_lane.distance)
                prev_lane = (ret_dist_trace[topic][-1].heading, ret_dist_trace[topic][-1].distance)
                if is_close(prev_lane, curr_lane):
                    continue  # Skip repeating ground truths
            if stamped_lane.stamp > progress_stamp + stamp_itvl:
                print("Finished processing "
                      "%.1f to %.1f second" % (progress_stamp/1000, (progress_stamp + stamp_itvl)/1000))
                progress_stamp += stamp_itvl
            ret_dist_trace[topic].append(stamped_lane)

    # Choose to use estimation from EST_TOPIC or CAM_IMG_TOPIC
    truth_list = ret_dist_trace[ODOM_TOPIC]
    if len(ret_dist_trace.get(CAM_IMG_TOPIC, ())) > 0:
        sorted_est_list = ret_dist_trace[CAM_IMG_TOPIC]
    else:
        sorted_est_list = []

    return file_name, (truth_list, sorted_est_list)


def main(argv: Any) -> None:
    try:
        gem_lanenet_path = pathlib.Path(rospkg.RosPack().get_path("gem_lanenet"))

    except rospkg.ResourceNotFound:
        warnings.warn("Cannot find ROS package 'gem_lanenet'."
                      "Try configure your ROS environment by `source devel/setup.bash`")
        return

    config_path = gem_lanenet_path / "configs/conf_polaris_gem.lanenet.yaml"
    weights_path = gem_lanenet_path / "lanenet_weights/tusimple_lanenet.ckpt"

    converter = ImageToLane(
        lanenet=LaneNetWLineFit(config_path=str(config_path),
                                weights_path=str(weights_path),
                                debug=False))

    TOPIC_TO_CB = {
        ODOM_TOPIC: odom_to_truth,
        CAM_IMG_TOPIC: converter.cam_img_to_lane
    }

    begin = time.time()
    sec_start = argv.start  # second
    sec_stop = argv.stop  # second
    file_samples_iter = (process_bag_file(bag_file_name, TOPIC_TO_CB, sec_start, sec_stop)
                         for bag_file_name in argv.bag_file)

    num_imgs = 0
    for file_name, truth_samples_list in file_samples_iter:
        num_imgs += len(truth_samples_list[1])
        with open("%s.%s-%s.pickle" % (file_name, str(sec_start).zfill(3), str(sec_stop).zfill(3)), 'wb') as f:
            pickle.dump(truth_samples_list, f)

    converter._lanenet.close()
    print("Image Process Time usage: %.2f seconds" % converter.time_usage)
    print("Total Time usage: %.2f seconds" % (time.time() - begin))
    print("Skipped %d out of %d images" % (converter.skip_count, num_imgs))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('bag_file', nargs='+', type=str)
    parser.add_argument(
        '--start', nargs='?', type=float, default=0.0, metavar='START',
        help="Processing only those messages with timestamp >= %(metavar)s second. (default: %(default)s)")
    parser.add_argument(
        '--stop', nargs='?', type=float, default=float('inf'), metavar='STOP',
        help="Processing only those messages with timestamp < %(metavar)s second. (default: %(default)s)")
    main(parser.parse_args())
