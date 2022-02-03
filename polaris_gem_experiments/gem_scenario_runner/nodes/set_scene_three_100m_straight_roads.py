#!/usr/bin/env python3

"""
Periodically setting new model poses drawn from a uniform distribution for the
world with three different straight roads.
"""
import itertools
from typing import Generator, NamedTuple

import numpy as np

import rospy
from geometry_msgs.msg import Pose, Point

from gem_scenario_runner import euler_to_quat, set_light_properties, set_model_pose, DIFFUSE_MAP


PLOT_NUM = 3
PLOT_SEP = 30.0  # meter
LANE_START, LANE_STOP = 2.0, 32.0  # meter
BOT_Z = -0.11  # meter


LaneDetectScene = NamedTuple("LaneDetectScene", [
    ("light_level", int),
    ("pose", Pose)
])  # TODO Put this in a Python package for reuse?


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
        set_light_properties(self.__light_name, scene.light_level)
        set_model_pose(self.__model_name, "world", scene.pose)


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
