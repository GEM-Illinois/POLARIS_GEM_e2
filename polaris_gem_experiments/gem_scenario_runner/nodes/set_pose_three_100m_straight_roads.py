#!/usr/bin/env python3

"""
Periodically setting new model poses drawn from a uniform distribution for the
world with three different straight roads.
"""
import itertools
from typing import Generator

import numpy as np
from scipy.spatial.transform import Rotation

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

PLOT_NUM = 3
PLOT_SEP = 30.0  # meter
LANE_START, LANE_STOP = 2.0, 32.0  # meter
BOT_Z = -0.11  # meter
AXES_SEQ = "ZYX"  # Convention for Euler angles. Follow REP-103
GZ_SET_MODEL_STATE = "/gazebo/set_model_state"


def quat_to_yaw(quat: Quaternion) -> float:
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    yaw, pitch, roll = Rotation.from_quat((x, y, z, w)).as_euler(AXES_SEQ)
    return yaw


def euler_to_quat(yaw, pitch=0.0, roll=0.0) -> Quaternion:
    quat = Rotation.from_euler(AXES_SEQ, (yaw, pitch, roll)).as_quat()
    return Quaternion(*quat)


def check_ground_truth(phi: float, cte: float, pose: Pose) -> bool:
    """ Check generated poses correspond to the given ground truth.
        Note that this is constructed so that there shouldn't be
        any floating point error.
    """
    # TODO change according to the new map
    return True


def gen_uniform_random_pose(phi: float, cte: float) -> Generator[Pose, None, None]:
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
        yield pose


def gen_vehicle_poses_from_truths(truth_iter, num_poses_each_truth: int) \
        -> Generator[Pose, None, None]:
    for phi, cte in truth_iter:
        yield from itertools.islice(gen_uniform_random_pose(phi, cte), num_poses_each_truth)


class TimedResetPose:
    """
    This class defines the callback function to reset a Model to new poses and
    is called periodically by rospy Timer class.
    """

    def __init__(self, model_name: str, gen_sample):
        self.__model_name = model_name
        self.__it = gen_sample

    def timer_callback(self, timer_ev: rospy.timer.TimerEvent) -> None:
        try:
            pose = next(self.__it)
        except StopIteration:
            rospy.signal_shutdown("No more poses")
            return
        self.set_model_pose(pose)

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


def main():
    rospy.init_node("set_model_pose", anonymous=True)

    model_name = rospy.get_param("~gazebo_model_name")
    reset_period = rospy.get_param("~reset_period")  # second
    truth_list = rospy.get_param("~truth_list", [])
    num_poses_each_truth = rospy.get_param("~num_poses_each_truth", 100)

    trp = TimedResetPose(
        model_name,
        gen_vehicle_poses_from_truths(truth_list, num_poses_each_truth)
    )

    rospy.Timer(rospy.Duration.from_sec(reset_period), trp.timer_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


if __name__ == "__main__":
    main()
