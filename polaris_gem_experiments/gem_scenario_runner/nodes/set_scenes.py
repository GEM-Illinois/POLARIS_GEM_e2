#!/usr/bin/env python3

"""
Periodically setting new model poses drawn from a uniform distribution for the selected world.
"""
import numpy as np

import rospy
from geometry_msgs.msg import Pose, Point

from gem_scenario_runner import euler_to_quat, get_uniform_random_light_level, \
    LaneDetectScene, TimedResetScene, GenSceneFromTruthBase, Percept


class GenScenesThreeStraightRoads(GenSceneFromTruthBase):
    PLOT_NUM = 3
    PLOT_SEP = 30.0  # meter
    LANE_START, LANE_STOP = 2.0, 32.0  # meter
    BOT_Z = -0.11  # meter

    @classmethod
    def _check_ground_truth(cls, psi: float, cte: float, pose: Pose) -> bool:
        """ Check generated poses correspond to the given ground truth.
            Note that this must be constructed so that there shouldn't be
            any floating point error.
        """
        # TODO change according to the new map
        return True

    def _get_scene_from_truth(self, truth: Percept) -> LaneDetectScene:
        """ Get a poses for a given ground truth psi and cte
        The samples are drawn first from a discrete choice among three lanes and
        then from a continuous uniform distribution between [LANE_START, LANE_STOP].

        Parameters
        ----------
        truth : tuple of float
            Heading angle error in radian and Cross track error range in meter
        """
        cte, psi = truth.offset, truth.yaw_err
        x = np.random.uniform(self.LANE_START, self.LANE_STOP)
        road_y = self.PLOT_SEP * np.random.choice(np.arange(self.PLOT_NUM))
        y = road_y - cte
        z = self.BOT_Z
        yaw = -psi
        rospy.loginfo("Sampled (x, y, yaw) = (%f, %f, %f) " % (x, y, yaw) +
                      "for ground truth (d, ψ) = (%f, %f)" % (cte, psi))
        pose = Pose(position=Point(x, y, z),
                    orientation=euler_to_quat(yaw=yaw))
        if not self._check_ground_truth(psi, cte, pose):
            rospy.logwarn("The pose does not map to the ground truth.")
        return LaneDetectScene(
            light_level=get_uniform_random_light_level(),
            pose=pose)


class GenScenesOneCurveRoad(GenSceneFromTruthBase):
    """ Following the convention of frames from REP 103, the forward direction of the vehicle aligns with x-axis.
        The left turn road is designed so that the center of all arcs are at (x, y) = (0, 100),
        and therefore the arc of the road is from (0, 0) to (100, 100) with radius 100.0m.
        Given the left lane width is 4.6 m, and the right lane width is 4.4 m by measuring the road model,
        the arc of the left lane is from (0, 2.3) to (97.7, 100) with radius 97.7m, and
        the arc of the right lane is from (0, -2.2) to (102.2, 100) with radius 102.2m.
        The width of each lane is measured by the distance between the mid of lane markers.
    """
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
    ARC_ANG_LB, ARC_ANG_UB = -np.pi / 2, 0.0  # radian
    ROAD_ARC_RADIUS = 100.0  # meters. The radius of road center line
    L_LANE_ARC_RADIUS = ROAD_ARC_RADIUS - (LANE_MARKER_PTS / 2 + L_LANE_PTS / 2) / METER_TO_PIXEL
    L_LANE_ARC_CURVATURE = 1 / L_LANE_ARC_RADIUS
    R_LANE_ARC_RADIUS = ROAD_ARC_RADIUS + (LANE_MARKER_PTS / 2 + R_LANE_PTS / 2) / METER_TO_PIXEL
    R_LANE_ARC_CURVATURE = 1 / R_LANE_ARC_RADIUS
    # endregion

    # region Constants to sample initial state
    ARC_ANG_START, ARC_ANG_STOP = np.deg2rad(-75), np.deg2rad(-45)  # radians
    assert ARC_ANG_LB <= ARC_ANG_START <= ARC_ANG_STOP <= ARC_ANG_UB
    # endregion

    @classmethod
    def _check_ground_truth(cls, psi: float, cte: float, pose: Pose) -> bool:
        """ Check generated poses correspond to the given ground truth.
            Note that this must be constructed so that there shouldn't be
            any floating point error.
        """
        # TODO change according to the new map
        return True

    def _get_scene_from_truth(self, truth: Percept) -> LaneDetectScene:
        """ Get a poses for a given ground truth psi and cte

        We uniformly sample an arc_ang in [ARC_ANG_START, ARC_ANG_STOP] to define the closest point to (x, y) on the arc
        (c + r*cos(arc_ang), c + r*sin(arc_ang)).

        Recall that for a left turn,
        offset = || (x, y) - c || - r
        yaw_err = (arc_ang + pi/2) - θ

        By rewriting the equations, we can derive a vehicle pose
        || (x, y) - c || = r + offset
        (x, y) = c + (r+offset)*(cos(arc_ang), sin(arc_ang))
        θ = (arc_ang + pi/2) - yaw_err

        Parameters
        ----------
        truth : tuple of float
            Heading angle error in radian and Cross track error range in meter
        """
        yaw_err, offset = truth.yaw_err, truth.offset

        r = self.L_LANE_ARC_RADIUS  # TODO select from lanes?
        arc_ang = np.random.uniform(self.ARC_ANG_START, self.ARC_ANG_STOP)
        x, y = np.array(self.ARC_CENTER) + (r + offset) * np.array([np.cos(arc_ang), np.sin(arc_ang)])
        z = self.BOT_Z
        yaw = (arc_ang + np.pi / 2) - yaw_err
        rospy.loginfo("Sampled (x, y, yaw) = (%f, %f, %f) " % (x, y, yaw) +
                      "for ground truth (d, ψ) = (%f, %f)" % (offset, yaw_err))
        pose = Pose(position=Point(x, y, z),
                    orientation=euler_to_quat(yaw=yaw))
        if not self._check_ground_truth(yaw_err, offset, pose):
            rospy.logwarn("The pose does not map to the ground truth.")
        return LaneDetectScene(
            light_level=get_uniform_random_light_level(),
            pose=pose)


def main():
    rospy.init_node("set_model_pose", anonymous=True)

    world_name = rospy.get_param("~gazebo_world_name")
    model_name = rospy.get_param("~gazebo_model_name")
    light_name = rospy.get_param("~light_name", "sundir")
    reset_period = rospy.get_param("~reset_period")  # second
    fields = rospy.get_param("~fields")
    truth_list = rospy.get_param("~truth_list", [])
    num_scenes_each_truth = rospy.get_param("~num_scenes_each_truth", 100)

    if "truth" not in fields or fields["truth"] != ["cte", "psi"]:
        raise ValueError("Unsupported field declaration %s" % fields)

    if world_name.endswith("three_100m_straight_roads.world"):
        true_percept_list = [Percept(yaw_err=psi, offset=cte, curvature=0.0) for cte, psi in truth_list]
        scene_generator = GenScenesThreeStraightRoads(true_percept_list, num_scenes_each_truth)
    elif world_name.endswith("one_100m_left_curved_road.world"):
        true_percept_list = [Percept(yaw_err=psi, offset=cte, curvature=GenScenesOneCurveRoad.L_LANE_ARC_CURVATURE)
                             for cte, psi in truth_list]
        scene_generator = GenScenesOneCurveRoad(true_percept_list, num_scenes_each_truth)
    else:
        raise ValueError("Unknown Gazebo world name %s" % world_name)

    trs = TimedResetScene(
        model_name, light_name,
        scene_generator
    )

    rospy.Timer(rospy.Duration.from_sec(reset_period), trs.timer_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


if __name__ == "__main__":
    main()
