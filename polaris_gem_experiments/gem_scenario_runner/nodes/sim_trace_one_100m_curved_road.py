#!/usr/bin/env python3
"""
Monte Carlo simulation lane centering on a left curved road in Gazebo

Following the convention of frames from REP 103, the forward direction of the vehicle aligns with x-axis.
The left turn road is designed so that the center of all arcs are at (x, y) = (0, 100),
and therefore the arc of the road is from (0, 0) to (100, 100) with radius 100.0m.
Given the left lane width is 4.6 m, and the right lane width is 4.4 m by measuring the road model,
the arc of the left lane is from (0, 2.3) to (97.7, 100) with radius 97.7m, and
the arc of the right lane is from (0, -2.2) to (102.2, 100) with radius 102.2m.
The width of each lane is measured by the distance between the mid of lane markers.
"""

from copy import deepcopy
import pathlib
import pickle
import time
from typing import List, Tuple

import numpy as np
from scipy.stats.distributions import truncnorm

import cv_bridge
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import Image
import rospy
from gem_lanenet.lanenet_w_line_fit import LaneNetWLineFit


from gem_scenario_runner import \
    euler_to_quat, pose_to_xy_yaw, control_pure_pursuit, dynamics, control_stanley, \
    get_uniform_random_light_level, set_light_properties, set_model_pose, pause_physics, unpause_physics, \
    LaneDetectScene

# region Constants for the world file
BOT_Z = -0.11  # meter  # Default elevation of the road

L_MARGIN_PTS = 80  # pixels
L_LANE_PTS = 440
R_LANE_PTS = 420
R_MARGIN_PTS = 100
LANE_MARKER_PTS = 20
ROAD_PTS = L_MARGIN_PTS + LANE_MARKER_PTS + L_LANE_PTS + LANE_MARKER_PTS + R_LANE_PTS + LANE_MARKER_PTS + R_MARGIN_PTS
METER_TO_PIXEL = 100.0  # 100 PIXELS is 1.0 meter

ROAD_WIDTH = ROAD_PTS / METER_TO_PIXEL
assert ROAD_WIDTH == 11.0

ARC_CENTER = (0.0, 100.0)
ARC_ANG_LB, ARC_ANG_UB = -np.pi/2, 0.0  # radian
ROAD_ARC_RADIUS = 100.0  # meters. The radius of road center line
L_LANE_ARC_RADIUS = ROAD_ARC_RADIUS - (LANE_MARKER_PTS / 2 + L_LANE_PTS / 2) / METER_TO_PIXEL
R_LANE_ARC_RADIUS = ROAD_ARC_RADIUS + (LANE_MARKER_PTS / 2 + R_LANE_PTS / 2) / METER_TO_PIXEL
# endregion

# region Constants to sample initial state
ARC_ANG_START, ARC_ANG_STOP = np.deg2rad(-75), np.deg2rad(-45)  # radians
assert ARC_ANG_LB <= ARC_ANG_START <= ARC_ANG_STOP <= ARC_ANG_UB
PHI_LIM = np.pi / 12  # radian. 15 degrees
CTE_LIM = 1.2  # meter
# endregion


def xy_yaw_to_truth(x, y, yaw) -> Tuple[float, float]:
    """
    Given the vehicle pose is (x, y, θ).
    Assuming a lane is defined by the center point c and radius r where r >> || (x, y) - c ||,
    and the closest point to (x, y) on the arc is defined by (c + r*cos(arc_ang), c + r*sin(arc_ang) ).
    where arc_ang can be derived from atan((y-c[1])/(x-c[0])).
    the ground truth (phi, cte) is related by the following.

    For left turn,
    offset = || (x, y) - c || - r
    yaw_err = (arc_ang + pi/2) - θ
    """
    lane_arc_radius = L_LANE_ARC_RADIUS  # TODO select from lanes?
    v_diff = np.array([x, y]) - np.array(ARC_CENTER)
    offset = np.linalg.norm(v_diff, ord=2) - lane_arc_radius
    yaw_err = np.arctan2(v_diff[1], v_diff[0]) + np.pi/2 - yaw
    return yaw_err, offset


def get_uniform_random_scene(yaw_err: float, offset: float) -> LaneDetectScene:
    """ Get a poses for a given ground truth phi and cte

    We uniformly sample an arc_ang in [ARC_ANG_START, ARC_ANG_STOP] to define the closest point to (x, y) on the arc
    (c + r*cos(arc_ang), c + r*sin(arc_ang)).

    Recall that for left turn,
    offset = || (x, y) - c || - r
    yaw_err = (arc_ang + pi/2) - θ

    By rewriting the equations, we can derive a vehicle pose
    || (x, y) - c || = r + offset
    (x, y) = c + (r+offset)*(cos(arc_ang), sin(arc_ang))
    θ = (arc_ang + pi/2) - yaw_err

    Parameters
    ----------
    yaw_err : float
        Heading angle error in radian
    offset : float
        Cross track error range in meter
    """
    r = L_LANE_ARC_RADIUS  # TODO select from lanes?
    arc_ang = np.random.uniform(ARC_ANG_START, ARC_ANG_STOP)
    x, y = np.array(ARC_CENTER) + (r+offset)*np.array([np.cos(arc_ang), np.sin(arc_ang)])
    z = BOT_Z
    yaw = (arc_ang + np.pi/2) - yaw_err
    rospy.loginfo("Sampled (x, y, yaw) = (%f, %f, %f) " % (x, y, yaw) +
                  "for ground truth (phi, cte) = (%f, %f)" % (yaw_err, offset))
    pose = Pose(position=Point(x, y, z),
                orientation=euler_to_quat(yaw=yaw))
    if not check_ground_truth(yaw_err, offset, pose):
        rospy.logwarn("The pose does not map to the ground truth.")
    return LaneDetectScene(
        light_level=get_uniform_random_light_level(),
        pose=pose)


def check_ground_truth(phi: float, cte: float, pose: Pose) -> bool:
    """ Check generated poses correspond to the given ground truth.
        Note that this is constructed so that there shouldn't be
        any floating point error.
    """
    # TODO
    return True


class ResetPose:
    """
    This class defines the function to reset a Model to a new pose.
    """

    def __init__(self, model_name: str, lanenet: LaneNetWLineFit):
        self.__model_name = model_name
        self.__light_name = "sundir"
        self._bridge = cv_bridge.CvBridge()
        self._lanenet = lanenet

        self._prev_perceived = (0.0, 0.0)

    def set_scene(self, scene: LaneDetectScene):
        set_light_properties(self.__light_name, scene.light_level)
        self.set_model_pose(scene.pose)

    def set_model_pose(self, pose: Pose) -> None:
        set_model_pose(self.__model_name, "world", pose)

    def perception(self, ros_img_msg: Image) -> Tuple[float, float]:
        curr_perceived = self._raw_perception(ros_img_msg)
        if not any(np.isnan(curr_perceived)):
            self._prev_perceived = curr_perceived
            return curr_perceived
        else:
            assert not any(np.isnan(self._prev_perceived))
            return self._prev_perceived

    def _raw_perception(self, ros_img_msg: Image):
        """ Given camera image, run LaneNet to estimate the heading and distance """

        cv_image = self._bridge.imgmsg_to_cv2(ros_img_msg, "bgr8")

        center_line, annotated_img = self._lanenet.detect(cv_image)
        if center_line is None:
            rospy.logwarn("Cannot infer the lane center line in the current image. Skip.")
            return np.nan, np.nan

        # calculate error w.r.t the chosen frame of reference of the vehicle (ego view).
        # NOTE coefficients are in the order of y = c[0] + c[1]*x (+ ... + c[n]*x^n)
        # where x-axis is the forward direction of the ego vehicle
        yaw_err = np.arctan(center_line.convert().coef[1])

        # Calculate the offset as the distance from the chosen frame to lane center line
        # NOTE In this simulation set up we always assume the perception is w.r.t rear_axle
        # base_footprint or base_link is the origin (0.0, 0.0)
        y_diff = center_line(0.0) - 0.0
        offset = y_diff * np.cos(yaw_err)
        return yaw_err, offset


def gen_init_truth_arr(num_traces: int, phi_bound: Tuple[float, float], cte_bound: Tuple[float, float]) \
        -> np.ndarray:
    phi_min, phi_max = phi_bound
    cte_min, cte_max = cte_bound
    my_mean, my_std = np.zeros(2), np.array([0.1, 0.4])
    myclip_a, myclip_b = np.array([phi_min, cte_min]), np.array([phi_max, cte_max])
    a, b = (myclip_a - my_mean) / my_std, (myclip_b - my_mean) / my_std
    return truncnorm.rvs(a, b, size=(num_traces, 2), loc=my_mean, scale=my_std)


class ImageBuffer:
    def __init__(self):
        self._curr_msg = None
        self._prev_data_hash = hash(None)

    def wait_and_get(self) -> Image:
        while self._curr_msg is None or hash(self._curr_msg.data) == self._prev_data_hash:
            pass
        msg = deepcopy(self._curr_msg)  # Must have local copy before getting hash
        self._prev_data_hash = hash(msg.data)
        return msg

    def cb(self, msg: Image) -> None:
        self._curr_msg = msg


def main() -> None:
    rospy.init_node("sim_traces", anonymous=True)

    model_name = rospy.get_param("~gazebo_model_name")
    controller = rospy.get_param("~controller", "pure_pursuit")
    config_path = rospy.get_param("~config_path")  # file path
    weights_path = rospy.get_param("~weights_path")  # file path
    out_dir = rospy.get_param("~out_dir", "")  # file path
    phi_min = rospy.get_param("~phi_min", -PHI_LIM)  # radian
    phi_max = rospy.get_param("~phi_max", PHI_LIM)  # radian
    cte_min = rospy.get_param("~cte_min", -CTE_LIM)  # meter
    cte_max = rospy.get_param("~cte_max", CTE_LIM)  # meter
    num_traces = rospy.get_param("~num_traces", 5)
    max_trace_len = rospy.get_param("~max_trace_len", 50)

    img_buffer = ImageBuffer()
    _ = rospy.Subscriber('front_single_camera/image_raw', Image, img_buffer.cb)

    lanenet_detect = LaneNetWLineFit(
        config_path=config_path,
        weights_path=weights_path)
    rp = ResetPose(model_name, lanenet_detect)

    init_truth_arr = gen_init_truth_arr(num_traces, (phi_min, phi_max), (cte_min, cte_max))
    rospy.sleep(0.001)  # Wait for the simulated clock to start

    traces = []  # type: List[Tuple[List[Tuple[float, float, float]], List[Tuple[float, float]]]]
    time_usage_img, time_usage_lanenet, time_usage_dynamics = 0.0, 0.0, 0.0
    try:
        for i, init_truth in enumerate(init_truth_arr):
            scene = get_uniform_random_scene(*init_truth)  # New random scene for each trace
            rp.set_scene(scene)
            rospy.loginfo("Trace #%d " % i +
                          "starting from (φ*, d*) = (%f, %f)" % tuple(init_truth))
            true_pose_list, perceived_list = [], []
            curr_pose = scene.pose
            true_pose_list.append(deepcopy(curr_pose))
            for _ in range(max_trace_len):
                start_gen_img = time.time()
                rp.set_model_pose(curr_pose)

                unpause_physics()
                ros_img_msg = img_buffer.wait_and_get()
                pause_physics()
                time_usage_img += time.time() - start_gen_img
                start_nnet = time.time()
                if True:
                    prcv_state = rp.perception(ros_img_msg)
                else:  # Perfect perception for debugging
                    prcv_state = xy_yaw_to_truth(*pose_to_xy_yaw(curr_pose))
                perceived_list.append(prcv_state)
                time_usage_lanenet += time.time() - start_nnet
                rospy.logdebug("Perceived state (φ, d) = (%f, %f)" % prcv_state)
                start_dynamics = time.time()

                if controller == "stanley":
                    next_pose = dynamics(curr_pose, control_stanley(prcv_state))
                elif controller == "pure_pursuit":
                    next_pose = dynamics(curr_pose, control_pure_pursuit(prcv_state), L_LANE_ARC_RADIUS)
                else:
                    raise RuntimeError

                time_usage_dynamics += time.time() - start_dynamics
                curr_pose = next_pose
                true_pose_list.append(deepcopy(curr_pose))

            # Convert from Gazebo pose to ground truth perception value after whole simulation trace
            truth_list = [pose_to_xy_yaw(pose) for pose in true_pose_list]
            traces.append((truth_list, perceived_list))
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    finally:
        rospy.loginfo("Generate images: %f seconds" % time_usage_img)
        rospy.loginfo("NNet Perception: %f seconds" % time_usage_lanenet)
        rospy.loginfo("Compute Next State: %f seconds" % time_usage_dynamics)
        out_path = pathlib.Path(out_dir)
        if out_path.is_dir():
            # Save x, y, yaw in case for sanity check
            time_str = time.strftime("%Y-%m-%d-%H-%M-%S")
            out_pickle_name = 'lanenet_%s_sim_traces_%s.xy_yaw.pickle' % (controller, time_str)
            out_pickle = out_path.joinpath(out_pickle_name)
            with out_pickle.open('wb') as f:
                pickle.dump(traces, f)
            # Save ground truth as well as perceived heading and distance
            converted_trace = [([xy_yaw_to_truth(*xy_yaw) for xy_yaw in xy_yaw_trace], prcv_trace)
                               for xy_yaw_trace, prcv_trace in traces]
            out_pickle_name = 'lanenet_%s_sim_traces_%s.bag.pickle' % (controller, time_str)
            out_pickle = out_path.joinpath(out_pickle_name)
            with out_pickle.open('wb') as f:
                pickle.dump(converted_trace, f)


if __name__ == "__main__":
    main()
