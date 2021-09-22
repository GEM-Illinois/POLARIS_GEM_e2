#!/usr/bin/env python3

from typing import NamedTuple, Tuple

import numpy as np
from numpy.linalg import norm
from scipy.spatial.transform import Rotation  # Use scipy instead because tf is deprecated

import rospy
from angles import shortest_angular_distance
from gem_lane_detect_msgs.msg import SimpleLaneStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

VehicleState2D = NamedTuple(
    "VehicleState2D",
    [("x", float), ("y", float), ("yaw", float), ("vx", float), ("vy", float)]
)


def quaternion_to_euler(quat: Quaternion) -> Tuple[float, float, float]:
    yaw, pitch, roll = Rotation.from_quat((quat.x, quat.y, quat.z, quat.w)).as_euler("ZYX")  # Follow REP-103
    return yaw, pitch, roll


def closest_waypoint_idx(ref_pos_arr: np.ndarray, pos_vec: np.ndarray) -> int:
    assert ref_pos_arr.shape[1] == 2
    # Equivalent to [(x-pos[0])**2 + (y-pos[1])**2 for x, y in wp_arr]
    dist_sq_arr = np.sum((ref_pos_arr - pos_vec) ** 2, axis=1)
    return int(np.argmin(dist_sq_arr))


class LaneDetectMock:
    WHEEL_BASE = 1.75  # meters

    def __init__(self, pub_lane: rospy.Publisher, ref_wp_arr: np.array) -> None:
        self._pub_lane = pub_lane
        self._ref_wp_arr = ref_wp_arr

    def odom_msg_cb(self, msg: Odometry) -> None:
        yaw, _, _ = quaternion_to_euler(msg.pose.pose.orientation)
        # This callback is called by the subscriber thread,
        # so we use a tuple to update all fields atomically.
        vehicle_state = VehicleState2D(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw=yaw,
            vx=msg.twist.twist.linear.x,
            vy=msg.twist.twist.linear.y,
        )

        front_axle_x = vehicle_state.x + self.WHEEL_BASE*np.cos(vehicle_state.yaw)
        front_axle_y = vehicle_state.y + self.WHEEL_BASE*np.sin(vehicle_state.yaw)

        front_axle_pos = np.array([front_axle_x, front_axle_y])
        # Use only x and y dimensions provided in the waypoints for now.
        ref_pos_arr = self._ref_wp_arr[:, 0:2]
        target_pos_idx = closest_waypoint_idx(ref_pos_arr, front_axle_pos)
        curr_target_pos = ref_pos_arr[target_pos_idx]
        next_target_pos = ref_pos_arr[(target_pos_idx + 1) % len(ref_pos_arr)]

        # Use current and next positions to approximate the tangent line and calculate target yaw
        vec_curr_to_next = next_target_pos - curr_target_pos
        target_seg_yaw = np.arctan2(vec_curr_to_next[1], vec_curr_to_next[0])

        # Offset is the (signed) distance from the front axle to the tangent line.
        # We use the projection onto the orthogonal vector of the tangent vector (aka vector rejection)
        vec_curr_to_next_rot90 = np.dot(np.array([[0.0, -1.0], [1.0, 0.0]]), vec_curr_to_next)
        vec_front_to_curr = curr_target_pos - front_axle_pos
        offset = np.dot(vec_front_to_curr, vec_curr_to_next_rot90) / norm(vec_curr_to_next)

        msg = SimpleLaneStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "front_wheel_axle"
        # Use shortest_angular_distance to normalized the angular difference in [-pi, pi]
        msg.lane.yaw_err = shortest_angular_distance(vehicle_state.yaw, target_seg_yaw)
        msg.lane.offset = offset
        msg.lane.curvature = np.nan  # TODO estimate curvature from nearby waypoints
        self._pub_lane.publish(msg)


def main():
    rospy.init_node("lane_detect_mock", anonymous=True)

    ref_wps_file = rospy.get_param("~ref_wps_file")

    pub_lane = rospy.Publisher("estimated_lane", SimpleLaneStamped, queue_size=1)

    ref_wp_arr = np.genfromtxt(ref_wps_file, delimiter=',')
    lane_detect = LaneDetectMock(pub_lane=pub_lane, ref_wp_arr=ref_wp_arr)

    _ = rospy.Subscriber("base_footprint/odom", Odometry, callback=lane_detect.odom_msg_cb, queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Lane Detect Mock node")


if __name__ == "__main__":
    main()
