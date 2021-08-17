#!/usr/bin/env python3

#================================================================
# File name: gem_odom.py                                                                  
# Description: show sensor info in Rviz                                                              
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 06/10/2021                                                                
# Date last modified: 07/02/2021                                                          
# Version: 0.1                                                                    
# Usage: roslaunch gem_gazebo gem_sensor_info.launch                                                                    
# Python version: 3.8                                                             
#================================================================

# Python Headers
import math
import random
import numpy as np

# ROS Headers
import rospy
from rospy import Time
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from tf import TransformBroadcaster

# Gazebo Headers
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from gazebo_msgs.msg import ModelState


def main():

    rospy.init_node('odom_pub')

    odom_pub = rospy.Publisher('/gem/odom', Odometry, queue_size=10)

    rospy.wait_for_service('/gazebo/get_model_state')

    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    odom   = Odometry()
    header = Header()
    header.frame_id = '/odom'

    model = GetModelStateRequest()
    model.model_name = 'gem'

    bc = TransformBroadcaster()

    r = rospy.Rate(20)

    translation = (0.0, 0.0, 0.0)
    rotation    = (0.0, 0.0, 0.0, 1.0)

    while not rospy.is_shutdown():

        result = get_model_srv(model)

        odom.pose.pose = result.pose
        odom.twist.twist = result.twist

        header.stamp = rospy.Time.now()
        odom.header = header

        odom_pub.publish(odom)

        r.sleep()

if __name__ == '__main__':
    main()