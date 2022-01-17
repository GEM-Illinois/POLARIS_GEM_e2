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
from typing import List, NamedTuple, Tuple

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats.distributions import truncnorm

import cv_bridge
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SetLightProperties
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty
import rospy
from gem_lanenet.lanenet_w_line_fit import LaneNetWLineFit


# region Constants for the world file
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
# R_LANE_ARC_RADIUS = ROAD_ARC_RADIUS + (LANE_MARKER_PTS / 2 + R_LANE_PTS / 2) / METER_TO_PIXEL

GZ_PAUSE_PHYSICS = "/gazebo/pause_physics"
GZ_UNPAUSE_PHYSICS = "/gazebo/unpause_physics"
GZ_SET_MODEL_STATE = "/gazebo/set_model_state"
GZ_SET_LIGHT_PROPERTIES = "/gazebo/set_light_properties"
# endregion

# region Constants for the vehicle
BOT_Z = -0.11  # meter
WHEEL_BASE = 1.75  # meter
LOOK_AHEAD = 6.0  # meter
STEER_LIM = 0.61  # radian
K_PP = 0.285  # Proportional Gain for Pure Pursuit
K_ST = 0.45  # Proportional Gain for Stanley
SPEED = 2.8  # meter/second
CYCLE_SEC = 0.1  # second
# endregion

# region Constants to sample initial state
ARC_ANG_START, ARC_ANG_STOP = np.deg2rad(-75), np.deg2rad(-45)  # radians
assert ARC_ANG_LB <= ARC_ANG_START <= ARC_ANG_STOP <= ARC_ANG_UB
PHI_LIM = np.pi / 12  # radian. 15 degrees
CTE_LIM = 1.2  # meter
DIFFUSE_MAP = {
    0: 0.7 * np.array([1.0, 0.95, 0.8, 1.0]),
    1: np.array([1.0, 0.95, 0.8, 1.0])
}

LaneDetectScene = NamedTuple("LaneDetectScene", [
    ("light_level", int),
    ("pose", Pose)
])
# endregion


def quat_to_yaw(quat: Quaternion) -> float:
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    yaw, pitch, roll = Rotation.from_quat((x, y, z, w)).as_euler("ZYX")
    return yaw


def euler_to_quat(yaw, pitch=0.0, roll=0.0) -> Quaternion:
    quat = Rotation.from_euler("ZYX", (yaw, pitch, roll)).as_quat()
    return Quaternion(*quat)


def pose_to_xy_yaw(pose: Pose) -> Tuple[float, float, float]:
    return pose.position.x, pose.position.y, quat_to_yaw(pose.orientation)


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
    yaw_err = (ARC_ANG + pi/2) - θ

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
        light_level=np.random.choice(len(DIFFUSE_MAP)),
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
        self._set_light_properties(scene.light_level)
        self.set_model_pose(scene.pose)

    def set_model_pose(self, pose: Pose) -> None:
        msg = ModelState()
        msg.model_name = self.__model_name
        msg.reference_frame = "world"  # Default is also world.
        msg.pose = pose

        rospy.wait_for_service(GZ_SET_MODEL_STATE)
        try:
            set_model_state_srv = \
                rospy.ServiceProxy(GZ_SET_MODEL_STATE, SetModelState)
            resp = set_model_state_srv(msg)

            # TODO Check response
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)

    def _set_light_properties(self, light_level: int) -> None:
        if light_level not in DIFFUSE_MAP:
            raise ValueError("Unsupported light level %d" % light_level)

        diffuse = DIFFUSE_MAP[light_level]
        rospy.wait_for_service(GZ_SET_LIGHT_PROPERTIES)
        try:
            set_light_properties_srv = \
                rospy.ServiceProxy(GZ_SET_LIGHT_PROPERTIES, SetLightProperties)
            resp = set_light_properties_srv(
                light_name=self.__light_name,
                cast_shadows=True,
                diffuse=ColorRGBA(*diffuse),
                specular=ColorRGBA(0.7, 0.7, 0.7, 1),
                attenuation_constant=0.9,
                attenuation_linear=0.01,
                attenuation_quadratic=0.001,
                direction=Vector3(-0.3, 0.4, -1.0),
                pose=Pose(position=Point(0, 0, 10), orientation=Quaternion(0, 0, 0, 1))
            )  # TODO Check response
            # TODO Check response
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)

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


def pause_physics():
    rospy.wait_for_service(GZ_PAUSE_PHYSICS)
    try:
        pause_physics_srv = \
            rospy.ServiceProxy(GZ_PAUSE_PHYSICS, Empty)
        resp = pause_physics_srv()
        # TODO Check response
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s" % e)


def unpause_physics():
    rospy.wait_for_service(GZ_UNPAUSE_PHYSICS)
    try:
        unpause_physics_srv = \
            rospy.ServiceProxy(GZ_UNPAUSE_PHYSICS, Empty)
        resp = unpause_physics_srv()
        # TODO Check response
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s" % e)


def gen_init_truth_arr(num_traces: int, phi_bound: Tuple[float, float], cte_bound: Tuple[float, float]) \
        -> np.ndarray:
    phi_min, phi_max = phi_bound
    cte_min, cte_max = cte_bound
    my_mean, my_std = np.zeros(2), np.array([0.1, 0.4])
    myclip_a, myclip_b = np.array([phi_min, cte_min]), np.array([phi_max, cte_max])
    a, b = (myclip_a - my_mean) / my_std, (myclip_b - my_mean) / my_std
    return truncnorm.rvs(a, b, size=(num_traces, 2), loc=my_mean, scale=my_std)


def control_pure_pursuit(prcv_state: Tuple[float, float]) -> float:
    yaw_err, offset = prcv_state
    heading, distance = -yaw_err, -offset

    # Let β be the angle between tangent line of curve path and the line from rear axle to look ahead point
    # based on cosine rule of triangles: cos(A) = (b^2+c^2-a^2)/2bc
    # cos(π/2+β) = (lookahead^2 + (-distance+radius)^2 - radius^2)/(2*lookahead*(-distance+radius))
    #            = -distance/lookahead + (lookahead^2 - distance^2)/(2*lookahead*(-distance+radius))
    #            ~ -distance/lookahead  # when radius==inf
    # Further, cos(π/2+β) = sinβ
    # -> β = arcsin(-distance/lookahead)
    radius = L_LANE_ARC_RADIUS  # TODO select from lanes or estimate by NNet?
    if np.isinf(radius):
        beta = np.arcsin(-distance/LOOK_AHEAD)
    elif abs(distance) <= LOOK_AHEAD:
        sin_beta = -distance/LOOK_AHEAD + (LOOK_AHEAD**2 - distance**2)/(2*LOOK_AHEAD*(-distance+radius))
        beta = np.arcsin(sin_beta)
    else:
        beta = -np.pi/2

    # calculate steering angle
    alpha = beta - heading
    angle_i = np.arctan((2 * K_PP * WHEEL_BASE * np.sin(alpha)) / LOOK_AHEAD)
    angle = angle_i * 2
    angle = np.clip(angle, -STEER_LIM, STEER_LIM)  # type: float  # no rounding performed
    return angle


def dynamics(curr_pose: Pose, steering: float) -> Pose:
    curr_x = curr_pose.position.x
    curr_y = curr_pose.position.y
    curr_z = curr_pose.position.z
    curr_yaw = quat_to_yaw(curr_pose.orientation)

    next_x = curr_x + SPEED * CYCLE_SEC * np.cos(curr_yaw)
    next_y = curr_y + SPEED * CYCLE_SEC * np.sin(curr_yaw)
    next_yaw = curr_yaw + SPEED * np.tan(steering) / WHEEL_BASE * CYCLE_SEC

    return Pose(position=Point(next_x, next_y, curr_z),
                orientation=euler_to_quat(yaw=next_yaw))


def control_stanley(prcv_state: Tuple[float, float]) -> float:
    yaw_err, offset = prcv_state
    heading, distance = -yaw_err, -offset

    # NOTE Convert to front axle assuming the lane is a line
    distance_f = distance + WHEEL_BASE*np.sin(heading)
    angle = -heading + np.arctan2(K_ST*-distance_f, SPEED)
    angle = np.clip(angle, -STEER_LIM, STEER_LIM)  # type: float  # no rounding performed
    return angle


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
    # init_truth_arr = np.array([(0, 0.3)])
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
                    next_pose = dynamics(curr_pose, control_pure_pursuit(prcv_state))
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
