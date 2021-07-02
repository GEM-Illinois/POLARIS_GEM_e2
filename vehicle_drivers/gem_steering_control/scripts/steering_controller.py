#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String, Bool, Float32, Float64
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd

# def speed_callback():
# 	pass

def controller():

	rospy.init_node('controller', anonymous=True)

	steer_pub  = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=10)
	gear_pub   = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=10)
	enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=10)

	# speed_sub  = rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, speed_callback)

	# Enable PACMod
	enable_cmd = Bool()
	enable_cmd.data = True

	# Gear control
	gear_cmd = PacmodCmd()
	gear_cmd.ui16_cmd = 2 # SHIFT_NEUTRAL

	# Controll steering wheel
	steer_cmd = PositionWithSpeed()
	# steer_cmd.angular_position       = 6.28 # radians, -: clockwise, +: counter-clockwise
	steer_cmd.angular_position       = 0 # radians, -: clockwise, +: counter-clockwise
	steer_cmd.angular_velocity_limit = 3.0   # radians/second

	steer_degrees = np.degrees(steer_cmd.angular_position)
	rospy.loginfo("Steering wheel position: %s", str(round(steer_degrees, 2)))

	rate = rospy.Rate(1)

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
