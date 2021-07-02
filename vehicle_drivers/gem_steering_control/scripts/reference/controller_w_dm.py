#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import String, Bool, Float32, Float64
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd
from pynput.keyboard import Key, Listener, KeyCode

#from controller_tests.msg import PID_input

steer_pub  = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
gear_pub   = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
accel_pub  = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
brake_pub  = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)

enabled    = False
accel_flag = False
gear_cmd   = PacmodCmd()
accel_cmd  = PacmodCmd()
brake_cmd  = PacmodCmd()

class PID_controller:

    def __init__(self, p=0.0, i=0.0, d=0.0, wg=20.0, avl=2.5, max_steering_angle=0.8, 
                 sp_p=0.0, sp_d=0.0, sp_i=0.0, sp_wg=0.70, desired_speed=0.0, max_accel=0.48):

        # Steering control parameters
        self.kp = p
        self.ki = i
        self.kd = d
        self.windup_guard = wg
        self.prev_error   = 0.0
        self.prev_time    = time.time()

        self.Pterm = 0.0
        self.Iterm = 0.0
        self.Dterm = 0.0

        self.max_steering_angle = max_steering_angle

        self.angular_velocity_limit = avl

        # Speed control parameters
        self.sp_kp = sp_p
        self.sp_kd = sp_d
        self.sp_ki = sp_i
        self.sp_windup_guard = sp_wg

        self.sp_Pterm = 0.0
        self.sp_Iterm = 0.0
        self.sp_Dterm = 0.0
        
        self.sp_prev_error = 0.0
        self.sp_prev_time  = time.time()
        self.desired_speed = desired_speed
        self.max_accel     = max_accel

        # Global messages

        global accel_cmd
        global brake_cmd

        accel_cmd.enable = False
        accel_cmd.clear  = True
        accel_cmd.ignore = True

        brake_cmd.enable = False
        brake_cmd.clear  = True
        brake_cmd.ignore = True
        

    def decision_making_input(self, data):

        global accel_flag
        global brake_cmd

        if data.data == 's':
            print('DECISION MAKING DECLARED UNSAFE: STOPPING')
            accel_flag = False
            brake_cmd.f64_cmd = 0.5
            brake_pub.publish(brake_cmd)
        

    def steer_control(self, data):

        current_time =  time.time()

        delta_time   = current_time - self.prev_time

        current_error = -(data.data - 1.3)

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

        output = -(self.kp*self.Pterm + self.ki*self.Iterm + self.kd*self.Dterm)

    	if output > self.max_steering_angle:
            output = self.max_steering_angle
        elif -output < -self.max_steering_angle:
            output = -self.max_steering_angle
     
        steer_cmd = PositionWithSpeed()
        steer_cmd.angular_position = output
        steer_cmd.angular_velocity_limit = self.angular_velocity_limit
        steer_pub.publish(steer_cmd)

    def speed_control(self, data):

        global accel_cmd

        if accel_flag:

            current_time  =  time.time()
            delta_time    = current_time-self.prev_time

            current_error = self.desired_speed - data.data
            delta_error   = current_error-self.sp_prev_error
            error_dot     = delta_error/delta_time

            self.sp_Pterm = current_error
            self.sp_Dterm = error_dot
            self.sp_Iterm += current_error*delta_time

            if self.sp_Iterm > self.sp_windup_guard:
                self.sp_Iterm = self.sp_windup_guard

            if self.sp_Iterm < -self.sp_windup_guard: 
                self.sp_Iterm = -self.sp_windup_guard

            self.prev_time = current_time
            self.sp_prev_error = current_error

            output = self.sp_kp*self.sp_Pterm + self.sp_kd*self.sp_Dterm + self.sp_ki*self.sp_Iterm
    
            
            print("PTERM")
            print(self.sp_kp*self.sp_Pterm)
            print(" ")
            print("VELOCITY")
            print(data.data)
            print(" ")
            print("ITERM")
            print(self.sp_ki*self.sp_Iterm)
            print(" ")
            print("DTERM")
            print(self.sp_kd*self.sp_Dterm)
            

            if output > self.max_accel:
                output = self.max_accel
            elif output < 0.31:
                output = 0.31

            accel_cmd.f64_cmd = output
            accel_pub.publish(accel_cmd)

        else:
            accel_cmd.f64_cmd = 0.0
            accel_pub.publish(accel_cmd)



def on_press(key):

    global enabled 
    global gear_cmd
    global accel_cmd
    global brake_cmd
    global accel_flag

    if key == KeyCode.from_char('f'):
        print('ACCELERATING')
        accel_flag = True
        brake_cmd.f64_cmd = 0.0

    if key == KeyCode.from_char('s'):
        print('STOPPING')
        accel_flag = False
        brake_cmd.f64_cmd = 0.5

    if key == Key.esc:
        return False

    if key == KeyCode.from_char('q'):
        print('DISENGAGED')
        enabled           = False
        accel_cmd.enable  = False
        accel_cmd.clear   = True
        accel_cmd.ignore  = True
        brake_cmd.enable  = False
        brake_cmd.clear   = True
        brake_cmd.ignore  = True
        gear_cmd.ui16_cmd = 2

    if key == KeyCode.from_char('p'):
        print('ENGAGED')
        enabled = True
        accel_cmd.enable = True
        accel_cmd.clear  = False
        accel_cmd.ignore = False
        brake_cmd.enable = True
        brake_cmd.clear  = False
        brake_cmd.ignore = False
        accel_cmd.f64_cmd = 0.0
        brake_cmd.f64_cmd = 0.5
        gear_cmd.ui16_cmd = 2

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
    brake_pub.publish(brake_cmd)

if __name__ == '__main__':

    #pid_controller = PID_controller(p=1.6, i=0, d=0.7, sp_p=1.35, sp_d=0.25, sp_i=0.08, desired_speed=0.3) #params for straight line
    pid_controller = PID_controller(p=6.1, i=0, d=2.7, sp_p=1.30, sp_d=0.02, sp_i=0.40, desired_speed=0.175) #params for curve
    rospy.init_node('PID_controller', anonymous=True)
    rospy.Subscriber("/lane_deviation", Float32, pid_controller.steer_control)
    rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, pid_controller.speed_control)
    #rospy.Subscriber("DECISIONMAKING", Char, pid_controller.decision_making_input)
    enable_pub.publish(Bool(False))
    listener = Listener(on_press=on_press)
    listener.start()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
