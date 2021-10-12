#!/usr/bin/env python3

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

PLOT_NUM = 3
PLOT_SEP = 30.0  # meter
HALF_LANE_WIDTH = 2.0  # meter
LANE_START, LANE_STOP = 2.0, 32.0  # meter
BOT_Z = -0.11  # meter

PHI_LIM = np.pi / 12  # radian. 15 degrees
CTE_LIM = 1.2  # meter
WHEEL_BASE = 1.75  # meter
LOOK_AHEAD = 6.0  # meter
STEER_LIM = 0.61  # radian
K_PP = 0.285  # Proportional Gain for Pure Pursuit
K_ST = 0.45  # Proportional Gain for Stanley
SPEED = 2.8  # meter/second
CYCLE_SEC = 0.1  # second

CURVE_PATH_RADIUS = np.inf

GZ_PAUSE_PHYSICS = "/gazebo/pause_physics"
GZ_UNPAUSE_PHYSICS = "/gazebo/unpause_physics"
GZ_SET_MODEL_STATE = "/gazebo/set_model_state"
GZ_SET_LIGHT_PROPERTIES = "/gazebo/set_light_properties"

DIFFUSE_MAP = {
    0: 0.7 * np.array([1.0, 0.95, 0.8, 1.0]),
    1: np.array([1.0, 0.95, 0.8, 1.0])
}

LaneDetectScene = NamedTuple("LaneDetectScene", [
    ("light_level", int),
    ("road_id", int),
    ("longitudinal_offset", float),
    ("pose", Pose)
])  # TODO Put this in a Python package for reuse?


def quat_to_yaw(quat: Quaternion) -> float:
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    yaw, pitch, roll = Rotation.from_quat((x, y, z, w)).as_euler("ZYX")
    return yaw


def euler_to_quat(yaw, pitch=0.0, roll=0.0) -> Quaternion:
    quat = Rotation.from_euler("ZYX", (yaw, pitch, roll)).as_quat()
    return Quaternion(*quat)


def check_ground_truth(phi: float, cte: float, pose: Pose) -> bool:
    """ Check generated poses correspond to the given ground truth.
        Note that this is constructed so that there shouldn't be
        any floating point error.
    """
    # TODO
    return True


def get_uniform_random_scene(phi: float, cte: float) -> LaneDetectScene:
    """ Get a poses for a given ground truth phi and cte
    The samples are drawn first from a discrete choice among three lanes and
    then from a continuous uniform distribution between [LANE_START, LANE_STOP].

    Parameters
    ----------
    phi : float
        Heading angle error in radian
    cte : float
        Cross track error range in meter
    """
    x = np.random.uniform(LANE_START, LANE_STOP)
    road_id = np.random.choice(PLOT_NUM)
    road_y = PLOT_SEP * road_id
    y = road_y - cte
    z = BOT_Z
    yaw = -phi
    rospy.loginfo("Sampled (x, y, yaw) = (%f, %f, %f) " % (x, y, yaw) +
                  "for ground truth (phi, cte) = (%f, %f)" % (phi, cte))
    pose = Pose(position=Point(x, y, z),
                orientation=euler_to_quat(yaw=yaw))
    if not check_ground_truth(phi, cte, pose):
        rospy.logwarn("The pose does not map to the ground truth.")
    return LaneDetectScene(
        light_level=np.random.choice(len(DIFFUSE_MAP)),
        road_id=road_id,
        longitudinal_offset=x,
        pose=pose)


def pose_to_xy_yaw(pose: Pose) -> Tuple[float, float, float]:
    return pose.position.x, pose.position.y, quat_to_yaw(pose.orientation)


class ResetPose:
    """
    This class defines the function to reset a Model to a new pose.
    """

    def __init__(self, model_name: str, lanenet: LaneNetWLineFit,
                 frame_id: str = "base_footprint"):
        self.__model_name = model_name
        self.__light_name = "sundir"
        self._bridge = cv_bridge.CvBridge()
        self._lanenet = lanenet
        self._frame_id = frame_id

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
        if self._frame_id in ["base_footprint", "base_link"]:
            # base_footprint or base_link is the origin (0.0, 0.0)
            y_diff = center_line(0.0) - 0.0
        elif self._frame_id == "front_wheel_axle":
            # front_wheel_axle is assumed at (WHEEL_BASE, 0.0) in meters.
            y_diff = center_line(WHEEL_BASE) - 0.0
        else:
            rospy.logwarn(
                "Unsupported frame_id %s for computing cross track error. " % self._frame_id
                + "Skipping.")
            return np.nan.np.nan
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
    # formula for calculating θ of goal point (same frame of reference as heading param)

    # Let β be the angle between tangent line of curve path and the line from rear axle to look ahead point
    # based on cosine rule of triangles: cos(A) = (b^2+c^2-a^2)/2bc
    # cos(π/2+β) = (lookahead^2 + (-distance+radius)^2 - radius^2)/(2*lookahead*(-distance+radius))
    #            = -distance/lookahead + (lookahead^2 - distance^2)/(2*lookahead*(-distance+radius))
    #            ~ -distance/lookahead  # when radius==inf
    # Further, cos(π/2+β) = sinβ
    # -> β = arcsin(-distance/lookahead)
    beta = np.arcsin(-distance/LOOK_AHEAD)
    # calculate steering angle
    alpha = beta - heading
    angle_i = np.arctan((2 * K_PP * WHEEL_BASE * np.sin(alpha)) / LOOK_AHEAD)
    angle = angle_i * 2
    angle = np.clip(angle, -STEER_LIM, STEER_LIM)  # type: float  # no rounding performed
    return angle


def dynamics_pure_pursuit(curr_pose: Pose, steering: float) -> Pose:
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
    angle = yaw_err + np.arctan2(K_ST*offset, SPEED)
    angle = np.clip(angle, -STEER_LIM, STEER_LIM)  # type: float  # no rounding performed
    return angle


def dynamics_stanley(curr_pose: Pose, steering: float) -> Pose:
    curr_x = curr_pose.position.x
    curr_y = curr_pose.position.y
    curr_z = curr_pose.position.z
    curr_yaw = quat_to_yaw(curr_pose.orientation)

    # Convert to front_wheel_axle
    curr_front_x = curr_x + WHEEL_BASE * np.cos(curr_yaw)
    curr_front_y = curr_y + WHEEL_BASE * np.sin(curr_yaw)

    next_front_x = curr_front_x + SPEED * CYCLE_SEC * np.cos(curr_yaw + steering)
    next_front_y = curr_front_y + SPEED * CYCLE_SEC * np.sin(curr_yaw + steering)
    next_yaw = curr_yaw + SPEED * np.sin(steering) / WHEEL_BASE * CYCLE_SEC

    # Convert back to rear_wheel_axle i.e. base_footprint
    next_x = next_front_x - WHEEL_BASE * np.cos(next_yaw)
    next_y = next_front_y - WHEEL_BASE * np.sin(next_yaw)

    return Pose(position=Point(next_x, next_y, curr_z),
                orientation=euler_to_quat(yaw=next_yaw))


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
    controller = rospy.get_param("~controller", "stanley")
    config_path = rospy.get_param("~config_path")  # file path
    weights_path = rospy.get_param("~weights_path")  # file path
    out_dir = rospy.get_param("~out_dir", "")  # file path
    phi_min = rospy.get_param("~phi_min", -PHI_LIM)  # radian
    phi_max = rospy.get_param("~phi_max", PHI_LIM)  # radian
    cte_min = rospy.get_param("~cte_min", -CTE_LIM)  # meter
    cte_max = rospy.get_param("~cte_max", CTE_LIM)  # meter
    num_traces = rospy.get_param("~num_traces", 5)
    max_trace_len = rospy.get_param("~max_trace_len", 50)

    if controller == "stanley":
        frame_id = "front_wheel_axle"
    elif controller == "pure_pursuit":
        frame_id = "base_footprint"
    else:
        rospy.logerr("Unknown controller name %s. Abort." % controller)
        return

    img_buffer = ImageBuffer()
    _ = rospy.Subscriber('front_single_camera/image_raw', Image, img_buffer.cb)

    lanenet_detect = LaneNetWLineFit(
        config_path=config_path,
        weights_path=weights_path)
    rp = ResetPose(model_name, lanenet_detect, frame_id=frame_id)

    init_truth_arr = gen_init_truth_arr(num_traces, (phi_min, phi_max), (cte_min, cte_max))
    rospy.sleep(0.001)  # Wait for the simulated clock to start

    traces = []  # type: List[Tuple[List[Tuple[float, float, float]], List[Tuple[float, float]]]]
    time_usage_img, time_usage_lanenet, time_usage_dynamics = 0.0, 0.0, 0.0
    try:
        for init_truth in init_truth_arr:
            scene = get_uniform_random_scene(*init_truth)  # New random scene for each trace
            rp.set_scene(scene)
            rospy.loginfo("New Trace starting from (φ*, d*) = (%f, %f)" % tuple(init_truth))
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
                prcv_state = rp.perception(ros_img_msg)
                perceived_list.append(prcv_state)
                time_usage_lanenet += time.time() - start_nnet
                rospy.loginfo("Perceived state (φ, d) = (%f, %f)" % prcv_state)
                start_dynamics = time.time()

                if controller == "stanley":
                    next_pose = dynamics_stanley(curr_pose, control_stanley(prcv_state))
                elif controller == "pure_pursuit":
                    next_pose = dynamics_pure_pursuit(curr_pose, control_pure_pursuit(prcv_state))
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
            out_pickle_name = 'lanenet_%s_sim_traces_%s.xy_yaw.pickle' % (controller, time.strftime("%Y-%m-%d-%H-%M-%S"))
            out_pickle = out_path.joinpath(out_pickle_name)
            with out_pickle.open('wb') as f:
                pickle.dump(traces, f)

            def xy_yaw_to_truth(x, y, yaw) -> Tuple[float, float]:
                yaw_err = -yaw
                offset = (-y + PLOT_SEP/2) % PLOT_SEP - PLOT_SEP/2
                return yaw_err, offset
            # Save ground truth as well as perceived heading and distance
            converted_trace = [([xy_yaw_to_truth(*xy_yaw) for xy_yaw in xy_yaw_trace], prcv_trace)
                               for xy_yaw_trace, prcv_trace in traces]
            out_pickle_name = 'lanenet_%s_sim_traces_%s.bag.pickle' % (controller, time.strftime("%Y-%m-%d-%H-%M-%S"))
            out_pickle = out_path.joinpath(out_pickle_name)
            with out_pickle.open('wb') as f:
                pickle.dump(converted_trace, f)


if __name__ == "__main__":
    main()
