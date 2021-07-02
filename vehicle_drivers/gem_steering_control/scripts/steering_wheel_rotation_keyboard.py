#!/usr/bin/env python3

"""
$ sudo pip3 install pynput
"""

import rospy
import time
import numpy as np
from std_msgs.msg import Char, String, Bool, Float32, Float64
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd
from pynput.keyboard import Key, Listener, KeyCode

enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
gear_pub   = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
brake_pub  = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
steer_pub  = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)

enabled    = False
gear_cmd   = PacmodCmd()
brake_cmd  = PacmodCmd()


class PID_controller:

    def __init__(self, p=0.0, i=0.0, d=0.0, wg=20.0, angular_velocity_limit=2.5, max_steering_angle=10.5):

        #Steering control parameters
        self.kp            = p
        self.ki            = i
        self.kd            = d
        self.windup_guard  = wg
        self.prev_error    = 0.0
        self.prev_time     = time.time()

        self.Pterm         = 0.0
        self.Iterm         = 0.0
        self.Dterm         = 0.0

        self.max_steering_angle     = max_steering_angle
        self.angular_velocity_limit = angular_velocity_limit

        # Global messages
        global accel_cmd
        global brake_cmd

        accel_cmd.enable = False
        accel_cmd.clear  = True
        accel_cmd.ignore = True

        brake_cmd.enable = False
        brake_cmd.clear  = True
        brake_cmd.ignore = True
       

    def steer_control(self, data):

        current_time  =  time.time()

        delta_time    = current_time - self.prev_time

        current_error = -(data.data - 1.3)

        delta_error   = current_error - self.prev_error
        error_dot     = delta_error/delta_time

        self.Pterm    = current_error
        self.Dterm    = error_dot
        self.Iterm   += current_error*delta_time

        if self.Iterm > self.windup_guard:
            self.Iterm = self.windup_guard

        if self.Iterm < -self.windup_guard: 
            self.Iterm = -self.windup_guard

        self.prev_time  = current_time
        self.prev_error = current_error

        output = -(self.kp*self.Pterm + self.ki*self.Iterm + self.kd*self.Dterm)

    	if output > self.max_steering_angle:
            output = self.max_steering_angle

        if output < -self.max_steering_angle:
            output = -self.max_steering_angle

        steer_cmd  = PositionWithSpeed()
        steer_cmd.angular_position       = output
        steer_cmd.angular_velocity_limit = self.angular_velocity_limit
        steer_pub.publish(steer_cmd)



def on_press(key):

    global enabled
    global gear_cmd
    global brake_cmd
    
    if key == Key.esc:
    	print("Exit program")
        return False

    if key == KeyCode.from_char('p'):
        print('ENGAGED')
        enabled           = True
        brake_cmd.enable  = True
        brake_cmd.clear   = False
        brake_cmd.ignore  = False
        brake_cmd.f64_cmd = 0.5
        gear_cmd.ui16_cmd = 2 # neutral position

    if key == KeyCode.from_char('q'):
        print('DISENGAGED')
        enabled           = False
        brake_cmd.enable  = False
        brake_cmd.clear   = True
        brake_cmd.ignore  = True
        gear_cmd.ui16_cmd = 2 # neutral position

    # if key == KeyCode.from_char('n'):
    #     print('GEAR: NEUTRAL')
    #     gear_cmd.ui16_cmd = 2

    # if key == KeyCode.from_char('d'):
    #     print('GEAR: FORWARD')
    #     gear_cmd.ui16_cmd = 3

    # if key == KeyCode.from_char('r'):
    # 	print('GEAR: REVERSE')
    #     gear_cmd.ui16_cmd = 1

    enable_pub.publish(Bool(enabled))
    gear_pub.publish(gear_cmd)
    brake_pub.publish(brake_cmd)


if __name__ == '__main__':

   # pid_controller = PID_controller(p=1.6, i=0, d=0.7, sp_p=1.35, sp_d=0.25, sp_i=0.08, desired_speed=0.3) #params for straight line

    pid_controller = PID_controller(p=6.1, i=0, d=2.7) # params for curve

    rospy.init_node('PID_controller', anonymous=True)

    rospy.Subscriber("/steering_value", Float32, pid_controller.steer_control)

    enable_pub.publish(Bool(False))

    listener = Listener(on_press=on_press)

    listener.start()
    
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)






# import rospy
# import time
# import numpy as np
# from std_msgs.msg import String, Bool, Float32, Float64
# from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd

# PI = 3.1415926535

# def speed_callback(msg):
# 	vehicle_speed = msg.data
# 	rospy.loginfo("Vehicle speed: %s", str(round(vehicle_speed, 2)))

# def controller():

# 	rospy.init_node('controller', anonymous=True)

# 	enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)

# 	steer_pub  = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
# 	gear_pub   = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
# 	# accel_pub  = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
# 	brake_pub  = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)

# 	speed_sub  = rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, speed_callback)

# 	enable_cmd = Bool()
# 	gear_cmd   = PacmodCmd()
# 	# accel_cmd  = PacmodCmd()
# 	brake_cmd  = PacmodCmd()
# 	steer_cmd  = PositionWithSpeed()

# 	rospy.loginfo("Disengaged!")
# 	enable_cmd.data                  = False
# 	# accel_cmd.enable                 = False
# 	# accel_cmd.clear                  = True
# 	# accel_cmd.ignore                 = True
# 	brake_cmd.enable                 = False
# 	brake_cmd.clear                  = True
# 	brake_cmd.ignore                 = True
# 	gear_cmd.ui16_cmd                = 2 # SHIFT_NEUTRAL
# 	steer_cmd.angular_position       = 0    # radians, -: clockwise, +: counter-clockwise
# 	steer_cmd.angular_velocity_limit = 2.0  # radians/second
# 	enable_pub.publish(enable_cmd)
# 	brake_pub.publish(brake_cmd)
# 	gear_pub.publish(gear_cmd)
# 	steer_pub.publish(steer_cmd)

# 	time.sleep(5)

# 	rospy.loginfo("Engaged!")
# 	enable_cmd.data                  = True
# 	# accel_cmd.enable                 = True
# 	# accel_cmd.clear                  = False
# 	# accel_cmd.ignore                 = False
# 	brake_cmd.enable                 = True
# 	brake_cmd.clear                  = False
# 	brake_cmd.ignore                 = False
# 	gear_cmd.ui16_cmd                = 2     # SHIFT_NEUTRAL
# 	steer_cmd.angular_position       = 2*PI  # radians, -: clockwise, +: counter-clockwise
# 	steer_cmd.angular_velocity_limit = 2.0   # radians/second

# 	steer_degrees = np.degrees(steer_cmd.angular_position)
# 	rospy.loginfo("Steering wheel position to : %s", str(round(steer_degrees, 2)))

# 	rate = rospy.Rate(1)

# 	while not rospy.is_shutdown():
# 		steer_pub.publish(steer_cmd)
# 		gear_pub.publish(gear_cmd)
# 		brake_pub.publish(brake_cmd)
# 		enable_pub.publish(enable_cmd)
# 		rate.sleep()

# if __name__ == '__main__':
# 	try:
# 		controller()
# 	except rospy.ROSInterruptException:
# 		print("exception")
# 		pass
