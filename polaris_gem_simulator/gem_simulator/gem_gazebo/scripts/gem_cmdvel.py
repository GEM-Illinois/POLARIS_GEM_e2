#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive

#================================================================
# File name: gem_cmdvel.py                                                                  
# Description: take care of GEM control in Gazebo                                                             
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 07/10/2021                                                                
# Date last modified: 07/12/2021                                                          
# Version: 0.1                                                                    
# Usage: roslaunch gem_gazebo gem_gazebo_rviz.launch                                                                     
# Python version: 3.8                                                             
#================================================================

def cmdvel_to_steering_angle(v, omega, steer_max, wheelbase):

    if (omega == 0 or v == 0):
        return 0

    # v: m/s
    # omega: radians/s

    if(omega > 0):
        radius = v / (omega * 1.0)
        return math.atan2(wheelbase, radius)
    else:
        omega = -omega
        radius = v / (omega * 1.0)
        return -math.atan2(wheelbase, radius)


def cmdvel_callback(msg):

    global ackermann_pub

    wheelbase = 1.75 # meters

    forward_v = msg.linear.x # m/s

    steer_max = 0.61

    steering = cmdvel_to_steering_angle(forward_v, msg.angular.z, steer_max, wheelbase)

    if (steering >= steer_max):
        steering = steer_max

    if (steering <= -steer_max):
        steering = -steer_max

    # steering_angle (radians)
    # steering_angle_velocity (radians/s)
    # speed (m/s)
    # acceleration (m/s^2)
    # jerk (m/s^3)
    msg = AckermannDrive()
    msg.steering_angle = steering
    msg.speed = forward_v
    
    ackermann_pub.publish(msg)


if __name__ == '__main__': 

    try:

        rospy.init_node('cmdvel_to_ackermann')
        
        cmdvel_sub = rospy.Subscriber('/gem/cmd_vel', Twist, cmdvel_callback, queue_size=1)

        ackermann_pub = rospy.Publisher('/gem/ackermann_cmd', AckermannDrive, queue_size=1)

        rospy.spin()
    
    except rospy.ROSInterruptException:

        pass