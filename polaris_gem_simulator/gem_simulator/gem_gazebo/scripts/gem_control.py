#!/usr/bin/env python3

# ================================================================
# File name: gem_control.py
# Description: Low level Ackermann steering for GEM in Gazebo
# Author: Hang Cui
# Email: hangcui3@illinois.edu
# Date created: 06/05/2021
# Date last modified: 10/03/2021
# Version: 0.1
# Usage: roslaunch gem_gazebo gem_gazebo_rviz.launch
# Python version: 3.8
# ================================================================

import math
import sys

import numpy as np
import threading

import tf2_ros
import rospy

from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64
from controller_manager_msgs.srv import ListControllers


PI = 3.141592653589739


def get_steer_angle(phi):
    if phi >= 0.0:
        return (PI / 2) - phi
    return (-PI / 2) - phi


def _get_front_wheel_params(prefix):
    steer_link = rospy.get_param(prefix + "steering_link_name")
    steer_controller = rospy.get_param(prefix + "steering_controller_name")
    wheel_controller = rospy.get_param(prefix + "axle_controller_name")
    diameter = float(rospy.get_param(prefix + "diameter"))
    return steer_link, steer_controller, wheel_controller, 1 / (PI * diameter)


def _get_rear_wheel_params(prefix):
    link = rospy.get_param(prefix + "link_name")
    wheel_controller = rospy.get_param(prefix + "axle_controller_name")
    diameter = float(rospy.get_param(prefix + "diameter"))
    return link, wheel_controller, 1 / (PI * diameter)


class GEMController(object):

    def __init__(self,
                 left_front_inv_circ, right_front_inv_circ,
                 left_rear_inv_circ, right_rear_inv_circ,
                 right_rear_link,
                 steer_joint_dist: float,
                 wheel_base: float,
                 left_steer_pub: rospy.Publisher, right_steer_pub: rospy.Publisher,
                 left_front_wheel_pub: rospy.Publisher, right_front_wheel_pub: rospy.Publisher,
                 left_rear_wheel_pub: rospy.Publisher, right_rear_wheel_pub: rospy.Publisher,
                 cmd_timeout: float = 0.5):
        self.left_front_inv_circ = left_front_inv_circ
        self.right_front_inv_circ = right_front_inv_circ
        self.left_rear_inv_circ = left_rear_inv_circ
        self.right_rear_inv_circ = right_rear_inv_circ

        self.right_rear_link = right_rear_link

        self.cmd_timeout = cmd_timeout

        self.prev_t = rospy.get_time()
        self.last_cmd_time = rospy.get_time()

        self.steer_joint_dist_div_2 = steer_joint_dist/2.0

        self.wheelbase = wheel_base

        self.wheelbase_inv = 1 / (self.wheelbase*1.0)

        self.wheelbase_sqr = self.wheelbase**2

        self.ackermann_cmd_lock = threading.Lock()

        self.steer_ang             = 0.0
        self.steer_ang_vel         = 0.0
        self.speed                 = 0.0
        self.accel                 = 0.0
        self.last_steer_ang        = 0.0
        self.theta_left            = 0.0
        self.theta_left_old        = 0.0
        self.theta_right           = 0.0
        self.theta_right_old       = 0.0
        self.last_speed            = 0.0
        self.last_accel_limit      = 0.0
        self.left_front_ang_vel    = 0.0
        self.right_front_ang_vel   = 0.0
        self.left_rear_ang_vel     = 0.0
        self.right_rear_ang_vel    = 0.0

        self.left_steer_pub        = left_steer_pub
        self.right_steer_pub       = right_steer_pub
        self.left_front_wheel_pub  = left_front_wheel_pub
        self.right_front_wheel_pub = right_front_wheel_pub
        self.left_rear_wheel_pub   = left_rear_wheel_pub
        self.right_rear_wheel_pub  = right_rear_wheel_pub

    def loop_body(self, curr_t):
        delta_t = curr_t - self.prev_t  # Calculate delta_t first
        self.prev_t = curr_t
        if self.cmd_timeout > 0.0 and (curr_t - self.last_cmd_time > self.cmd_timeout):

            steer_ang_changed, center_y = self.control_steering(self.last_steer_ang, 0.0, 0.001)

            self.control_wheels(0.0, 0.0, 0.0, steer_ang_changed, center_y)

        elif delta_t > 0.0:

            with self.ackermann_cmd_lock:
                steer_ang     = self.steer_ang
                steer_ang_vel = self.steer_ang_vel
                speed         = self.speed
                accel         = self.accel

            steer_ang_changed, center_y = self.control_steering(steer_ang, steer_ang_vel, delta_t)

            self.control_wheels(speed, accel, delta_t, steer_ang_changed, center_y)

        self.left_steer_pub.publish(self.theta_left)
        self.right_steer_pub.publish(self.theta_right)

        if self.left_front_wheel_pub:
            self.left_front_wheel_pub.publish(self.left_front_ang_vel)

        if self.right_front_wheel_pub:
            self.right_front_wheel_pub.publish(self.right_front_ang_vel)

        if self.left_rear_wheel_pub:
            self.left_rear_wheel_pub.publish(self.left_rear_ang_vel)

        if self.right_rear_wheel_pub:
            self.right_rear_wheel_pub.publish(self.right_rear_ang_vel)

    def ackermann_callback(self, ackermann_cmd):
        self.last_cmd_time = rospy.get_time()
        with self.ackermann_cmd_lock:
            self.steer_ang = ackermann_cmd.steering_angle
            self.steer_ang_vel = ackermann_cmd.steering_angle_velocity
            self.speed = ackermann_cmd.speed
            self.accel = ackermann_cmd.acceleration

    def control_steering(self, steer_ang, steer_ang_vel_limit, delta_t):

        if steer_ang_vel_limit > 0.0:
            ang_vel = (steer_ang - self.last_steer_ang) / delta_t
            ang_vel = max(-steer_ang_vel_limit, min(ang_vel, steer_ang_vel_limit))
            theta = self.last_steer_ang + ang_vel * delta_t
        else:
            theta = steer_ang

        center_y = self.wheelbase * math.tan((PI/2)-theta)

        steer_ang_changed = theta != self.last_steer_ang

        if steer_ang_changed:
            self.last_steer_ang = theta
            self.theta_left = get_steer_angle(math.atan(self.wheelbase_inv * (center_y - self.steer_joint_dist_div_2)))
            self.theta_right = get_steer_angle(math.atan(self.wheelbase_inv * (center_y + self.steer_joint_dist_div_2)))

        return steer_ang_changed, center_y

    def control_wheels(self, speed, accel_limit, delta_t, steer_ang_changed, center_y):

        if accel_limit > 0.0:
            self.last_accel_limit = accel_limit
            accel = (speed - self.last_speed) / delta_t
            accel = max(-accel_limit, min(accel, accel_limit))
            veh_speed = self.last_speed + accel * delta_t
        else:
            self.last_accel_limit = accel_limit
            veh_speed = speed

        if veh_speed != self.last_speed or steer_ang_changed:
            self.last_speed = veh_speed
            left_dist = center_y - self.steer_joint_dist_div_2
            right_dist = center_y + self.steer_joint_dist_div_2
            gain = (2 * PI) * veh_speed / abs(center_y)
            r = math.sqrt(left_dist ** 2 + self.wheelbase_sqr)
            self.left_front_ang_vel = gain * r * self.left_front_inv_circ
            r = math.sqrt(right_dist ** 2 + self.wheelbase_sqr)
            self.right_front_ang_vel = gain * r * self.right_front_inv_circ
            gain = (2 * PI) * veh_speed / center_y
            self.left_rear_ang_vel = gain * left_dist * self.left_rear_inv_circ
            self.right_rear_ang_vel = gain * right_dist * self.right_rear_inv_circ


def _vector3_to_numpy(msg) -> np.ndarray:
    return np.array([msg.x, msg.y, msg.z])


def main(argv):
    rospy.init_node("gem_ackermann_controller")
    (left_steer_link, left_steer_controller, left_front_wheel_controller, left_front_inv_circ) = \
        _get_front_wheel_params("~left_front_wheel/")
    (right_steer_link, right_steer_controller, right_front_wheel_controller, right_front_inv_circ) = \
        _get_front_wheel_params("~right_front_wheel/")
    (left_rear_link, left_rear_wheel_controller, left_rear_inv_circ) = \
        _get_rear_wheel_params("~left_rear_wheel/")
    (right_rear_link, right_rear_wheel_controller, right_rear_inv_circ) = \
        _get_rear_wheel_params("~right_rear_wheel/")
    publishing_frequency = rospy.get_param("~publishing_frequency", 30.0)
    cmd_timeout = rospy.get_param("~cmd_timeout", 0.5)

    # Look up frame translations w.r.t right rear wheel
    tf_buffer = tf2_ros.Buffer()
    _ = tf2_ros.TransformListener(tf_buffer)
    t_point = rospy.Time(0)  # Time point, 0 means get the latest
    timeout = rospy.Duration(secs=30)
    try:
        tf_buffer.can_transform(left_steer_link, right_rear_link, t_point, timeout)
        trans = tf_buffer.lookup_transform(left_steer_link, right_rear_link, t_point)
        ls = _vector3_to_numpy(trans.transform.translation)

        tf_buffer.can_transform(right_steer_link, right_rear_link, t_point, timeout)
        trans = tf_buffer.lookup_transform(right_steer_link, right_rear_link, t_point)
        rs = _vector3_to_numpy(trans.transform.translation)

        tf_buffer.can_transform(left_rear_link, right_rear_link, t_point, timeout)
        trans = tf_buffer.lookup_transform(left_rear_link, right_rear_link, t_point)
        lrw = _vector3_to_numpy(trans.transform.translation)
        rrw = np.array([0.0] * 3)

        steer_joint_dist = np.linalg.norm(ls - rs)
        wheel_base = np.linalg.norm((ls + rs) / 2.0 - (lrw + rrw) / 2.0)
        rospy.logdebug("Wheel base: %.3f m, Steer joint Distance: %.3f m" % (wheel_base, steer_joint_dist))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Cannot lookup frames using TF2 after %.1f seconds. Shutting down." % timeout.to_sec())
        return

    list_controllers = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
    list_controllers.wait_for_service()
    list_controllers.close()

    # Explicitly create Publisher and Subscriber in main function
    controller = GEMController(
        left_front_inv_circ, right_front_inv_circ,
        left_rear_inv_circ, right_rear_inv_circ,
        right_rear_link,
        steer_joint_dist=steer_joint_dist,
        wheel_base=wheel_base,
        left_steer_pub=rospy.Publisher(left_steer_controller + "/command", Float64, queue_size=1),
        right_steer_pub=rospy.Publisher(right_steer_controller + "/command", Float64, queue_size=1),
        left_front_wheel_pub=rospy.Publisher(left_front_wheel_controller + "/command", Float64, queue_size=1),
        right_front_wheel_pub=rospy.Publisher(right_front_wheel_controller + "/command", Float64, queue_size=1),
        left_rear_wheel_pub=rospy.Publisher(left_rear_wheel_controller + "/command", Float64, queue_size=1),
        right_rear_wheel_pub=rospy.Publisher(right_rear_wheel_controller + "/command", Float64, queue_size=1),
        cmd_timeout=cmd_timeout
    )
    _ = rospy.Subscriber("ackermann_cmd", AckermannDrive, controller.ackermann_callback, queue_size=1)

    try:
        rate = rospy.Rate(hz=publishing_frequency)
        while not rospy.is_shutdown():
            curr_t = rospy.get_time()
            controller.loop_body(curr_t)
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down GEM Ackermann Controller")


if __name__ == "__main__":
    sys.exit(main(rospy.myargv()) or 0)
