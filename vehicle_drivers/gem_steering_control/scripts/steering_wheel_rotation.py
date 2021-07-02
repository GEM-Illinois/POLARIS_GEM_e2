#!/usr/bin/env python3

import rospy
import time
import numpy as np
from std_msgs.msg import String, Bool, Float32, Float64
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd

PI = 3.1415926535

def speed_callback(msg):
	vehicle_speed = msg.data
	rospy.loginfo("Vehicle speed: %s", str(round(vehicle_speed, 2)))

def controller():

	rospy.init_node('controller', anonymous=True)

	enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)

	steer_pub  = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
	gear_pub   = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
	# accel_pub  = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
	brake_pub  = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)

	speed_sub  = rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, speed_callback)

	enable_cmd = Bool()
	gear_cmd   = PacmodCmd()
	# accel_cmd  = PacmodCmd()
	brake_cmd  = PacmodCmd()
	steer_cmd  = PositionWithSpeed()

	rospy.loginfo("Disengaged!")
	enable_cmd.data                  = False
	# accel_cmd.enable                 = False
	# accel_cmd.clear                  = True
	# accel_cmd.ignore                 = True
	brake_cmd.enable                 = False
	brake_cmd.clear                  = True
	brake_cmd.ignore                 = True
	gear_cmd.ui16_cmd                = 2 # SHIFT_NEUTRAL
	steer_cmd.angular_position       = 0    # radians, -: clockwise, +: counter-clockwise
	steer_cmd.angular_velocity_limit = 2.0  # radians/second
	enable_pub.publish(enable_cmd)
	brake_pub.publish(brake_cmd)
	gear_pub.publish(gear_cmd)
	steer_pub.publish(steer_cmd)

	time.sleep(5)

	rospy.loginfo("Engaged!")
	enable_cmd.data                  = True
	# accel_cmd.enable                 = True
	# accel_cmd.clear                  = False
	# accel_cmd.ignore                 = False
	brake_cmd.enable                 = True
	brake_cmd.clear                  = False
	brake_cmd.ignore                 = False
	gear_cmd.ui16_cmd                = 2     # SHIFT_NEUTRAL
	steer_cmd.angular_position       = 2*PI  # radians, -: clockwise, +: counter-clockwise
	steer_cmd.angular_velocity_limit = 2.0   # radians/second

	steer_degrees = np.degrees(steer_cmd.angular_position)
	rospy.loginfo("Steering wheel position to : %s", str(round(steer_degrees, 2)))

	rate = rospy.Rate(1)

	while not rospy.is_shutdown():
		steer_pub.publish(steer_cmd)
		gear_pub.publish(gear_cmd)
		brake_pub.publish(brake_cmd)
		enable_pub.publish(enable_cmd)
		rate.sleep()

if __name__ == '__main__':
	try:
		controller()
	except rospy.ROSInterruptException:
		print("exception")
		pass
