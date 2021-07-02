#!/usr/bin/env python3

import rospy
import time
import numpy as np
from std_msgs.msg import String, Bool, Float32, Float64
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd

"""
Input: -35 to 35 degrees

"""
def front2steer(front_angle):
	if (front_angle > 0):
		steer_angle = round(-0.1084*front_angle**2 + 21.775*front_angle, 2)
		# rospy.loginfo("Steering wheel angle: %s" % str(steer_angle))
		return np.radians(steer_angle)
	elif (front_angle < 0):
		front_angle = -front_angle
		steer_angle = round(-0.1084*front_angle**2 + 21.775*front_angle, 2)
		# rospy.loginfo("Steering wheel angle: %s" % str(-steer_angle))
		return -np.radians(steer_angle)
	else:
		# rospy.loginfo("Steering wheel angle: %s" % str(0))
		return 0

flip_flag   = 0  
front_angle = 0  # degrees
steer_angle = 0  # radians

def front_controller():

	global front_angle
	global steer_angle
	global flip_flag

	rospy.init_node('front_angle_test', anonymous=True)

	front_steer_pub  = rospy.Publisher('/pacmod/as_rx/front_steer_cmd', PositionWithSpeed, queue_size=1)

	front_steer_cmd  = PositionWithSpeed()
	front_steer_cmd.angular_position       = 0  # radians, -: clockwise, +: counter-clockwise
	front_steer_cmd.angular_velocity_limit = 0

	rate = rospy.Rate(2) # 1Hz

	while not rospy.is_shutdown():

		front_steer_cmd.angular_position = steer_angle

		front_steer_pub.publish(front_steer_cmd)

		if (flip_flag == 0): # increase value up to 40 degrees

			if (front_angle < 35):
				front_angle += 0.5
				# rospy.loginfo("Frontal wheel angle: %s" % str(front_angle))
				steer_angle = front2steer(front_angle) # radians
			else:
				flip_flag = 1

		else: # decrease value up to -40 degrees

			if (front_angle > -35):
				front_angle -= 0.5
				# rospy.loginfo("Frontal wheel angle: %s" % str(front_angle))
				steer_angle = front2steer(front_angle) # radians
			else:
				flip_flag = 0

		rate.sleep()

if __name__ == '__main__':
	try:
		front_controller()
	except rospy.ROSInterruptException:
		print("exception")
		pass
