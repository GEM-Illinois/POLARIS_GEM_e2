#!/usr/bin/env python3

import math
from typing import NamedTuple

from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
import rospy

from gem_lane_detect_msgs.msg import SimpleLaneStamped

SimpleLaneRepr = NamedTuple(
    "SimpleLaneRepr",
    [("yaw_err", float), ("offset", float), ("curvature", float)]
)


class PurePursuitLaneCentering:
    DESIRED_F_VEL = 2.8  # m/s
    LOOK_HEAD = 6.0  # m
    WHEEL_BASE = 1.75  # m
    STEERING_MIN = -0.61
    STEERING_MAX = 0.61

    def __init__(self, pub_ackermann: rospy.Publisher):
        self._pub_ackermann = pub_ackermann

        self._lane = SimpleLaneRepr(0.0, 0.0, 0.0)
        self._f_vel = 0.0

    def lane_msg_cb(self, msg: SimpleLaneStamped) -> None:
        # This callback is called by the subscriber thread,
        # so we use a tuple to update all fields atomically.
        # TODO store and use timestamp?
        self._lane = SimpleLaneRepr(yaw_err=msg.lane.yaw_err,
                                    offset=msg.lane.offset,
                                    curvature=msg.lane.curvature)

    def odom_msg_cb(self, msg: Odometry) -> None:
        x_vel = msg.twist.twist.linear.x
        y_vel = msg.twist.twist.linear.y
        self._f_vel = math.sqrt(x_vel**2 + y_vel**2)

    def loop_body(self) -> None:
        f_vel = self._f_vel
        theta_e = self._lane.yaw_err
        ef = self._lane.offset

        alpha = theta_e + math.asin(ef / self.LOOK_HEAD)
        delta = math.atan(2 * self.WHEEL_BASE * math.sin(alpha) / self.LOOK_HEAD)

        if delta > self.STEERING_MAX:
            delta = self.STEERING_MAX
        elif delta < self.STEERING_MIN:
            delta = self.STEERING_MIN

        rospy.logdebug("Forward velocity %g m/s, Cross track Error: %g m, Heading Error: %g deg" %
                       (f_vel, ef, math.degrees(theta_e)))

        ackermann_msg = AckermannDrive()
        ackermann_msg.speed = self.DESIRED_F_VEL
        ackermann_msg.steering_angle = delta
        self._pub_ackermann.publish(ackermann_msg)


def main():
    """ The ROS node, all publishers, and all subscribers are initialized in main
        function to explicitly count the number of threads and avoid unexpected
        concurrent behavior.
    """
    rospy.init_node("pure_pursuit_lane_centering", anonymous=True)

    # Initialize publishers
    pub_ackermann = rospy.Publisher("ackermann_cmd", AckermannDrive, queue_size=1)

    lane_centering = PurePursuitLaneCentering(pub_ackermann)

    # Register callback function
    _ = rospy.Subscriber("estimated_lane", SimpleLaneStamped,
                         callback=lane_centering.lane_msg_cb, queue_size=1)
    _ = rospy.Subscriber("base_footprint/odom", Odometry, callback=lane_centering.odom_msg_cb, queue_size=1)

    try:
        rate = rospy.Rate(hz=20)
        while not rospy.is_shutdown():
            rate.sleep()
            lane_centering.loop_body()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Stanley Lane Centering node")


if __name__ == '__main__':
    main()
