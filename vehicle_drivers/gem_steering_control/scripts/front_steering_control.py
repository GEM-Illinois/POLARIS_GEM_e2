#!/usr/bin/python3

import rospy
import time
import numpy as np
from std_msgs.msg import String, Bool, Float32, Float64
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd


front_angle = 0  # degrees
# steer_angle = 0  # radians

"""
Input: -35 to 35 degrees

"""
def front2steer(f_angle):
	if (f_angle > 0):
		steer_angle = round(-0.1084*f_angle**2 + 21.775*f_angle, 2)
		# rospy.loginfo("Steering wheel angle: %s" % str(steer_angle))
		return np.radians(steer_angle)
	elif (f_angle < 0):
		f_angle = -front_angle
		steer_angle = round(-0.1084*f_angle**2 + 21.775*f_angle, 2)
		# rospy.loginfo("Steering wheel angle: %s" % str(-steer_angle))
		return -np.radians(steer_angle)
	else:
		# rospy.loginfo("Steering wheel angle: %s" % str(0))
		return 0

def steer_callback(msg):
	global front_angle
	# front steering angle from image processing
	front_angle = msg.data

def front_controller():

	global front_angle

	rospy.init_node('front_angle_control', anonymous=True)

	image_sub        = rospy.Subscriber("/front_steering_angle", Float64, steer_callback)
	front_steer_pub  = rospy.Publisher("/pacmod/as_rx/front_steer_cmd", PositionWithSpeed, queue_size=1)

	front_steer_cmd                        = PositionWithSpeed()
	front_steer_cmd.angular_position       = 0  # radians, -: clockwise, +: counter-clockwise
	front_steer_cmd.angular_velocity_limit = 0

	rate = rospy.Rate(5) # 5Hz

	while not rospy.is_shutdown():

		steer_angle = front2steer(front_angle) # radians

		rospy.loginfo("Steering wheel angle: " + str(round(np.degrees(steer_angle),2)))

		front_steer_cmd.angular_position = steer_angle

		front_steer_pub.publish(front_steer_cmd)

		rate.sleep()

if __name__ == '__main__':
	try:
		front_controller()
	except rospy.ROSInterruptException:
		print("exception")
		pass
