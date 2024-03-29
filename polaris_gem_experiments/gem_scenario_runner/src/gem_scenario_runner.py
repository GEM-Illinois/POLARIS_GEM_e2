import abc
from typing import NamedTuple, Iterable, Any, Sequence

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SetLightProperties
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from scipy.spatial.transform import Rotation
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty

GZ_PAUSE_PHYSICS = "/gazebo/pause_physics"
GZ_UNPAUSE_PHYSICS = "/gazebo/unpause_physics"
GZ_SET_MODEL_STATE = "/gazebo/set_model_state"
GZ_SET_LIGHT_PROPERTIES = "/gazebo/set_light_properties"
DIFFUSE_MAP = {
    0: 0.7 * np.array([1.0, 0.95, 0.8, 1.0]),
    1: np.array([1.0, 0.95, 0.8, 1.0])
}

# region Constants for the vehicle
WHEEL_BASE = 1.75  # meter
LOOK_AHEAD = 6.0  # meter
STEER_LIM = 0.61  # radian
K_PP = 0.285  # Proportional Gain for Pure Pursuit
K_ST = 0.45  # Proportional Gain for Stanley
SPEED = 2.8  # meter/second
CYCLE_SEC = 0.1  # second
# endregion

LaneDetectScene = NamedTuple("LaneDetectScene", [
    ("light_level", int),
    ("pose", Pose)
])

Percept = NamedTuple("Percept", [
    ("yaw_err", float),
    ("offset", float),
    ("curvature", float)
])

State = NamedTuple("State", [
    ("x", float),
    ("y", float),
    ("yaw", float)
])


def state_list_to_ndarray(state_list: Sequence[State]) -> np.ndarray:
    return np.fromiter(state_list,
                       dtype=[('x', float), ('y', float), ('yaw', float)])


def percept_list_to_ndarray(percept_list: Sequence[Percept]) -> np.ndarray:
    return np.fromiter(percept_list,
                       dtype=[('psi', float), ('cte', float), ('curvature', float)])


def quat_to_yaw(quat: Quaternion) -> float:
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    # ZYX is the axis convention for Euler angles. Follow REP-103
    yaw, pitch, roll = Rotation.from_quat((x, y, z, w)).as_euler("ZYX")
    return yaw


def euler_to_quat(yaw, pitch=0.0, roll=0.0) -> Quaternion:
    quat = Rotation.from_euler("ZYX", (yaw, pitch, roll)).as_quat()
    return Quaternion(*quat)


def pose_to_xy_yaw(pose: Pose) -> State:
    return State(pose.position.x, pose.position.y, quat_to_yaw(pose.orientation))


def get_uniform_random_light_level() -> int:
    return np.random.choice(len(DIFFUSE_MAP))


class GenSceneFromTruthBase(abc.ABC):
    def __init__(self, truth_iter: Iterable[Percept], num_scenes_each_truth: int) -> None:
        self.__truth_it = iter(truth_iter)
        self.__num_scenes_per_truth = num_scenes_each_truth

    def __iter__(self):
        return (self._get_scene_from_truth(truth)
                for truth in self.__truth_it for _ in range(self.__num_scenes_per_truth))

    @abc.abstractmethod
    def _get_scene_from_truth(self, truth: Any) -> LaneDetectScene:
        raise NotImplementedError


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


def set_model_pose(model_name:str, reference_frame: str, pose: Pose):
    msg = ModelState()
    msg.model_name = model_name
    msg.reference_frame = reference_frame
    msg.pose = pose

    rospy.wait_for_service(GZ_SET_MODEL_STATE)
    try:
        set_model_state_srv = \
            rospy.ServiceProxy(GZ_SET_MODEL_STATE, SetModelState)
        resp = set_model_state_srv(msg)
        # TODO Check response
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s" % e)


def set_light_properties(light_name, light_level: int) -> None:
    if light_level not in DIFFUSE_MAP:
        raise ValueError("Unsupported light level %d" % light_level)

    diffuse = DIFFUSE_MAP[light_level]
    rospy.wait_for_service(GZ_SET_LIGHT_PROPERTIES)
    try:
        set_light_properties_srv = \
            rospy.ServiceProxy(GZ_SET_LIGHT_PROPERTIES, SetLightProperties)
        resp = set_light_properties_srv(
            light_name=light_name,
            cast_shadows=True,
            diffuse=ColorRGBA(*diffuse),
            specular=ColorRGBA(0.7, 0.7, 0.7, 1),
            attenuation_constant=0.9,
            attenuation_linear=0.01,
            attenuation_quadratic=0.001,
            direction=Vector3(-0.3, 0.4, -1.0),
            pose=Pose(position=Point(0, 0, 10), orientation=Quaternion(0, 0, 0, 1))
        )
        # TODO Check response
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s" % e)


def control_pure_pursuit(prcv_state: Percept) -> float:
    yaw_err, offset = prcv_state.yaw_err, prcv_state.offset
    radius = np.inf if prcv_state.curvature == 0 else 1.0 / prcv_state.curvature
    heading, distance = -yaw_err, -offset

    # Let β be the angle between tangent line of curve path and the line from rear axle to look ahead point
    # based on cosine rule of triangles: cos(A) = (b^2+c^2-a^2)/2bc
    # cos(π/2+β) = (lookahead^2 + (-distance+radius)^2 - radius^2)/(2*lookahead*(-distance+radius))
    #            = -distance/lookahead + (lookahead^2 - distance^2)/(2*lookahead*(-distance+radius))
    #            ~ -distance/lookahead  # when radius==inf
    # Further, cos(π/2+β) = sinβ
    # -> β = arcsin(-distance/lookahead)
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


def control_stanley(prcv_state: Percept) -> float:
    yaw_err, offset = prcv_state.yaw_err, prcv_state.offset
    heading, distance = -yaw_err, -offset

    # NOTE Convert to front axle assuming the lane is a line
    distance_f = distance + WHEEL_BASE*np.sin(heading)
    angle = -heading + np.arctan2(K_ST*-distance_f, SPEED)
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


class TimedResetScene:
    """
    This class defines the callback function to reset a Model to new poses as
    well as other world properties such as light level and direction.
    The callback is called periodically by rospy Timer class.
    """

    def __init__(self, model_name: str, light_name: str, gen_sample: Iterable[LaneDetectScene]):
        self.__model_name = model_name
        self.__light_name = light_name
        self.__it = iter(gen_sample)

    def timer_callback(self, timer_ev: rospy.timer.TimerEvent) -> None:
        try:
            scene = next(self.__it)
        except StopIteration:
            rospy.signal_shutdown("No more poses")
            return
        self.set_scene(scene)

    def set_scene(self, scene: LaneDetectScene) -> None:
        set_light_properties(self.__light_name, scene.light_level)
        set_model_pose(self.__model_name, "world", scene.pose)
