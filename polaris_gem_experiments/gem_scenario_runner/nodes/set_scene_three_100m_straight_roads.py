#!/usr/bin/env python3

"""
Periodically setting new model poses drawn from a uniform distribution for the
world with three different straight roads.
"""
import itertools
from typing import Generator, NamedTuple

import numpy as np
from scipy.spatial.transform import Rotation

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SetLightProperties
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA

PLOT_NUM = 3
PLOT_SEP = 30.0  # meter
LANE_START, LANE_STOP = 2.0, 32.0  # meter
BOT_Z = -0.11  # meter

GZ_SET_MODEL_STATE = "/gazebo/set_model_state"
GZ_SET_LIGHT_PROPERTIES = "/gazebo/set_light_properties"

DIFFUSE_MAP = {
    0: 0.7 * np.array([1.0, 0.95, 0.8, 1.0]),
    1: np.array([1.0, 0.95, 0.8, 1.0])
}

LaneDetectScene = NamedTuple("LaneDetectScene", [
    ("light_level", int),
    ("pose", Pose)
])  # TODO Put this in a Python package for reuse?


def quat_to_yaw(quat: Quaternion) -> float:
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    # Convention for Euler angles. Follow REP-103
    yaw, pitch, roll = Rotation.from_quat((x, y, z, w)).as_euler("ZYX")
    return yaw


def euler_to_quat(yaw, pitch=0.0, roll=0.0) -> Quaternion:
    # Convention for Euler angles. Follow REP-103
    quat = Rotation.from_euler("ZYX", (yaw, pitch, roll)).as_quat()
    return Quaternion(*quat)


def check_ground_truth(phi: float, cte: float, pose: Pose) -> bool:
    """ Check generated poses correspond to the given ground truth.
        Note that this is constructed so that there shouldn't be
        any floating point error.
    """
    # TODO change according to the new map
    return True


def gen_uniform_random_scene(phi: float, cte: float) -> Generator[LaneDetectScene, None, None]:
    """ Generate poses for a given ground truth phi and cte
    The samples are drawn first from a discrete choice among three lanes and
    then from a continuous uniform distribution between [LANE_START, LANE_STOP].

    Parameters
    ----------
    phi : float
        Heading angle error in radian
    cte : float
        Cross track error range in meter
    """
    while True:
        x = np.random.uniform(LANE_START, LANE_STOP)
        road_y = PLOT_SEP * np.random.choice(np.arange(PLOT_NUM))
        y = road_y - cte
        z = BOT_Z
        yaw = -phi
        rospy.loginfo("Sampled (x, y, yaw) = (%f, %f, %f) " % (x, y, yaw) +
                      "for ground truth (phi, cte) = (%f, %f)" % (phi, cte))
        pose = Pose(position=Point(x, y, z),
                    orientation=euler_to_quat(yaw=yaw))
        if not check_ground_truth(phi, cte, pose):
            rospy.logwarn("The pose does not map to the ground truth.")
        yield LaneDetectScene(
            light_level=np.random.choice(len(DIFFUSE_MAP)),
            pose=pose
        )


def gen_scenes_from_truths(truth_iter, num_scenes_each_truth: int) \
        -> Generator[LaneDetectScene, None, None]:
    for phi, cte in truth_iter:
        yield from itertools.islice(gen_uniform_random_scene(phi, cte), num_scenes_each_truth)


class TimedResetScene:
    """
    This class defines the callback function to reset a Model to new poses as
    well as other world properties such as light level and direction.
    The callback is called periodically by rospy Timer class.
    """

    def __init__(self, model_name: str, light_name: str, gen_sample):
        self.__model_name = model_name
        self.__light_name = light_name
        self.__it = gen_sample

    def timer_callback(self, timer_ev: rospy.timer.TimerEvent) -> None:
        try:
            scene = next(self.__it)
        except StopIteration:
            rospy.signal_shutdown("No more poses")
            return
        self.set_scene(scene)

    def set_scene(self, scene: LaneDetectScene) -> None:
        self._set_light_properties(scene.light_level)
        self._set_model_state(scene.pose)

    def _set_light_properties(self, light_level: int) -> None:
        if light_level not in DIFFUSE_MAP:
            raise ValueError("Unsupported light level %d" % light_level)

        # Set lighting
        diffuse = DIFFUSE_MAP[light_level]
        rospy.wait_for_service(GZ_SET_LIGHT_PROPERTIES)
        try:
            set_light_properties_srv = \
                rospy.ServiceProxy(GZ_SET_LIGHT_PROPERTIES, SetLightProperties)
            # TODO Change light pose and direction
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
            pass
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)

    def _set_model_state(self, pose: Pose) -> None:
        # Teleport vehicle model
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


def main():
    rospy.init_node("set_model_pose", anonymous=True)

    model_name = rospy.get_param("~gazebo_model_name")
    light_name = rospy.get_param("~light_name", "sundir")
    reset_period = rospy.get_param("~reset_period")  # second
    truth_list = rospy.get_param("~truth_list", [])
    num_scenes_each_truth = rospy.get_param("~num_scenes_each_truth", 100)

    trs = TimedResetScene(
        model_name, light_name,
        gen_scenes_from_truths(truth_list, num_scenes_each_truth)
    )

    rospy.Timer(rospy.Duration.from_sec(reset_period), trs.timer_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


if __name__ == "__main__":
    main()
