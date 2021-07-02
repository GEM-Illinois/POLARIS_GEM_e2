#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String, Bool, Float32, Float64
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd


angular_position = 0

def f_steer_callback(msg):
	global angular_position
	angular_position = msg.angular_position  # radians, -: clockwise, +: counter-clockwise

def controller():

	global angular_position

	rospy.init_node('controller', anonymous=True)

	front_steer_sub  = rospy.Subscriber('/pacmod/as_rx/front_steer_cmd', PositionWithSpeed, f_steer_callback)
	steer_pub        = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=10)
	gear_pub         = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=10)
	enable_pub       = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=10)

	# Enable PACMod
	enable_cmd = Bool()
	enable_cmd.data = True

	# Gear control
	gear_cmd = PacmodCmd()
	gear_cmd.ui16_cmd = 2 # SHIFT_NEUTRAL

	# Controll steering wheel
	steer_cmd = PositionWithSpeed()
	steer_cmd.angular_position       = angular_position # radians, -: clockwise, +: counter-clockwise
	steer_cmd.angular_velocity_limit = 2.0   # radians/second

	# steer_degrees = np.degrees(steer_cmd.angular_position)
	# rospy.loginfo("Steering wheel position: %s", str(round(steer_degrees, 2)))

	rate = rospy.Rate(5)

	while not rospy.is_shutdown():
		steer_pub.publish(steer_cmd)
		gear_pub.publish(gear_cmd)
		enable_pub.publish(enable_cmd)
		rate.sleep()

if __name__ == '__main__':
	try:
		controller()
	except rospy.ROSInterruptException:
		print("exception")
		pass
