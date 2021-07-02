#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String, Bool, Float32, Float64
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd


def controller():

	rospy.init_node('controller', anonymous=True)

	enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=10)

	# Enable PACMod
	enable_cmd = Bool()
	enable_cmd.data = False
	enable_pub.publish(enable_cmd)
	

if __name__ == '__main__':
	try:
		controller()
	except rospy.ROSInterruptException:
		print("exception")
		pass
