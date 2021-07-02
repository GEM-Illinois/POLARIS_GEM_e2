#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import String, Bool, Float32
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd
#from controller_tests.msg import PID_input
from pynput.keyboard import Key, Listener, KeyCode

steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=20)
gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
#steer_pub = rospy.Publisher('steering_command', PositionWithSpeed, queue_size=20)
enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=20)
accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)

enabled = False
gear_cmd = PacmodCmd()
accel_cmd = PacmodCmd()
brake_cmd = PacmodCmd()

class PID_controller:

    def __init__(self, p=0.0, i=0.0, d=0.0, wg=20.0, avl=0.5):
        self.kp = p
        self.ki = i
        self.kd = d
        self.windup_guard = wg
        self.prev_error = 0.0
        self.prev_time = time.time()

        self.Pterm = 0.0
        self.Iterm = 0.0
        self.Dterm = 0.0

        self.angular_velocity_limit = avl
        
        global accel_cmd
        global brake_cmd
        accel_cmd.enable = False
        accel_cmd.clear = True
        accel_cmd.ignore = True
        brake_cmd.enable = False
        brake_cmd.clear = True
        brake_cmd.ignore = True
        

    def control(self, data):
        current_time =  time.time()
        delta_time = current_time-self.prev_time

        current_error = data.data
        delta_error = current_error-self.prev_error
        error_dot = delta_error/delta_time

        self.Pterm = current_error
        self.Dterm = error_dot
        self.Iterm += current_error*delta_time
        if self.Iterm > self.windup_guard:
            self.Iterm = self.windup_guard
        if self.Iterm < -self.windup_guard: 
            self.Iterm = -self.windup_guard

        self.prev_time = current_time
        self.prev_error = current_error

        output = self.kp*self.Pterm + self.ki*self.Iterm + self.kd*self.Dterm 

        print(output)

        steer_cmd = PositionWithSpeed()
        steer_cmd.angular_position = 0.0 #output
        steer_cmd.angular_velocity_limit = self.angular_velocity_limit
        steer_pub.publish(steer_cmd)


def on_press(key):
    global enabled 
    global gear_cmd
    global accel_cmd
    global brake_cmd

    if key == KeyCode.from_char('f'):
        print('forward velocity set to 1')
        accel_cmd.f64_cmd = 0.32
        brake_cmd.f64_cmd = 0.0
    if key == KeyCode.from_char('s'):
        print('forward velocity set to 0')
        accel_cmd.f64_cmd = 0.0
        brake_cmd.f64_cmd = 0.5
    if key == Key.esc:
        return False
    if key == KeyCode.from_char('q'):
        print('DISENGAGED')
        enabled = False
        accel_cmd.enable = False
        accel_cmd.clear = True
        accel_cmd.ignore = True
        brake_cmd.enable = False
        brake_cmd.clear = True
        brake_cmd.ignore = True
    if key == KeyCode.from_char('p'):
        print('ENGAGED')
        enabled = True
        accel_cmd.enable = True
        accel_cmd.clear = False
        accel_cmd.ignore = False
        brake_cmd.enable = True
        brake_cmd.clear = False
        brake_cmd.ignore = False
        accel_cmd.f64_cmd = 0.0
        brake_cmd.f64_cmd = 0.5
    if key == KeyCode.from_char('n'):
        print('GEAR: NEUTRAL')
        gear_cmd.ui16_cmd = 2
    if key == KeyCode.from_char('d'):
        print('GEAR: DRIVE')
        gear_cmd.ui16_cmd = 3
    if key == KeyCode.from_char('r'):
        print('GEAR: REVERSE')
        gear_cmd.ui16_cmd = 1

    enable_pub.publish(Bool(enabled))
    gear_pub.publish(gear_cmd)
    print(brake_cmd)
    accel_pub.publish(accel_cmd)
    brake_pub.publish(brake_cmd)

if __name__ == '__main__':

    pid_controller = PID_controller(p=0.4, i=0, d=0.9)
    rospy.init_node('PID_controller', anonymous=True)
    rospy.Subscriber("/lane_deviation", Float32, pid_controller.control)
    enable_pub.publish(Bool(False))
    listener = Listener(on_press=on_press)
    listener.start()
    while not rospy.core.is_shutdown():
        #enable_pub.publish(Bool(enabled))
        rospy.rostime.wallsleep(0.5)
