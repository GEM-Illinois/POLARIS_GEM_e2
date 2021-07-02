#!/usr/bin/env python3

from __future__ import print_function

import sys
import copy
import time
import rospy
import rospkg

import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
# from scipy.linalg import expm, logm

from std_msgs.msg import String, Float64
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

from poly_fit import *

PI = 3.141592653589793

class ImageConverter:

    def __init__(self):

        self.node_name = "gem_vision"
        
        rospy.init_node(self.node_name)
        
        rospy.on_shutdown(self.cleanup)
        
        # self.cv2_window_name = self.node_name
        # cv2.namedWindow(self.cv2_window_name, cv2.WINDOW_NORMAL)

        self.last_time = time.time()
        
        # Create the cv_bridge object
        self.bridge          = CvBridge()

        self.image_sub       = rospy.Subscriber("/mako_1/mako_1/image_raw", Image, self.image_callback)
        self.image_pub_debug = rospy.Publisher("/front_camera/image_warped", Image, queue_size=1)  
        self.image_pub       = rospy.Publisher("/front_camera/image_processed", Image, queue_size=1)
        self.steer_pub       = rospy.Publisher("/front_steering_angle", Float64, queue_size=1)

        self.mtx             = np.array([[1046.17936,   0.0,          1011.65344],
                                         [0.0,          1051.07316,   737.852696],
                                         [0.0,          0.0,          1.0       ]])

        self.dist            = np.array([[-0.2331957, 0.09314091, -0.0004832, 0.00042556, -0.02042143]])

        self.M               = np.array([[-6.01973802e-01, -1.76961914e+00,  1.70241948e+03], \
                                         [ 4.40319476e-03, -2.51130209e+00,  1.82805476e+03], \
                                         [ 4.07703219e-06, -1.67184264e-03,  1.00000000e+00]])

        self.M_inv           = np.array([[ 3.58609432e-01, -7.08481114e-01,  6.84638587e+02], \
                                         [ 2.00709642e-03, -4.00725621e-01,  7.29131457e+02], \
                                         [ 1.89348719e-06, -6.67061682e-04,  1.00000000e+00]])

        self.f_steer_prev = 0
        self.f_steer_curr = 0        

        self.frame_width  = 1920 
        self.frame_height = 1080 

        rospy.loginfo("Waiting for image topics...")

    def image2vehicle(self, ploty, center_fitx):

        x_car_frame = (center_fitx - 1058)/233
        y_car_frame = (1623 - ploty)/113
    
        # look-ahead distance
        ld = 6.5
        L = 1.75  # [m] wheel base of vehicle
    
        d  = 0
        i = len(x_car_frame)
        index = i
    
        while(d<ld):
            i = i - 1
            d = np.sqrt(x_car_frame[i]**2 + y_car_frame[i]**2)
            index = i

        alpha = np.pi/2 - np.arctan2(y_car_frame[index], x_car_frame[index])
    
        # front steering angle
        delta = np.arctan2(2.0 * L * np.sin(alpha) / ld, 1.0)
        
        return index, np.rad2deg(delta)

    def process_image(self, undistorted, warped_binary):

        # curvature_params[0]: image_shape
        # curvature_params[1]: ploty
        # curvature_params[2]: center_fit
        # curvature_params[3]: center_fitx
        # input: binary bird-view image
        curvature_params = fit_poly_curvature(warped_binary)
        
        # 
        ld_point_idx, front_steering_angle = self.image2vehicle(curvature_params[1], curvature_params[3])

        # output: left lane curvature, right lane curvature, 
        center_curverad, offset = measure_curvature_real(curvature_params[0], curvature_params[1], 
                                                         curvature_params[2], curvature_params[3])

        predict = drawing(undistorted, warped_binary, curvature_params[1], curvature_params[3], self.M_inv, ld_point_idx)
        
        return predict, front_steering_angle, center_curverad

    def pipeline(self, img, r_thresh=(245,255), s_thresh=(240,255), sx_thresh=(60,100)):

        # BGR image, r channel is good for white lane
        r_channel = img[:,:,1]
        r_binary  = np.zeros_like(r_channel)
        r_binary[(r_channel >= r_thresh[0]) & (r_channel <= r_thresh[1])] = 1

        # Convert to HLS color space and separate the V channel
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        l_channel = hls[:,:,1]
        s_channel = hls[:,:,2]

        # Sobel x, take the derivative in x
        sobelx = cv2.Sobel(l_channel, cv2.CV_64F, 1, 0) 

        # Absolute x derivative to accentuate lines away from horizontal
        abs_sobelx = np.absolute(sobelx) 
        scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
    
        # Threshold x gradient
        sxbinary = np.zeros_like(scaled_sobel)
        sxbinary[(scaled_sobel >= sx_thresh[0]) & (scaled_sobel <= sx_thresh[1])] = 1
    
        # Threshold color channel
        s_binary = np.zeros_like(s_channel)
        s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 1

        # Combine the two binary thresholds
        combined_binary = np.zeros_like(sxbinary)
        combined_binary[(s_binary == 1) | (r_binary == 1) | (sxbinary == 1)] = 1

        return combined_binary

    def image_callback(self, ros_image):

        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # BGR
        undistorted = cv2.undistort(frame, self.mtx, self.dist, None, self.mtx)

        # BGR, birdview image
        warped = cv2.warpPerspective(undistorted, self.M, (undistorted.shape[1], undistorted.shape[0]), flags=cv2.INTER_LINEAR)

        # BGR color binary 
        warped_binary = self.pipeline(warped)


        # Calculate frame rate
        fps_txt = "FPS: " + str(int(1/(time.time()-self.last_time)))
        
        # Process the frame using the process_image() function
        pub_image, front_steering_angle, center_curverad = self.process_image(undistorted, warped_binary)

        self.f_steer_curr = front_steering_angle

        # # if wrong predict angle occurs
        # if(np.abs(self.f_steer_curr-self.f_steer_prev)>2):
        #     self.f_steer_curr = self.f_steer_prev

        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(pub_image, fps_txt, (60, 60), font, 2, (255,0,0), 2)
        cv2.putText(pub_image,'Lane Curvature: %.3f m' % (center_curverad), (60, 130), font, 2, (0,255,0), 2, cv2.LINE_AA)
        cv2.putText(pub_image,'Front Steering Angle: %.3f degrees' % (self.f_steer_curr), (60, 200), \
                    font, 2, (0,255,0), 2, cv2.LINE_AA)

        steer_angle_msg = Float64()
        steer_angle_msg.data = self.f_steer_curr

        self.f_steer_prev = self.f_steer_curr

        self.last_time = time.time()
        
        # cv2.imshow(self.node_name, img_undisted)
        # # Process any keyboard commands
        # self.keystroke = cv2.waitKey(5)
        # if 32 <= self.keystroke and self.keystroke < 128:
        #     cc = chr(self.keystroke).lower()
        #     if cc == 'q':
        #         # The user has press the q key, so exit
        #         rospy.signal_shutdown("User hit q key to quit.")
                
        try:
            # Convert OpenCV image to ROS image and publish
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
            self.image_pub_debug.publish(self.bridge.cv2_to_imgmsg(warped, "bgr8"))
            self.steer_pub.publish(steer_angle_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def cleanup(self):
        print ("Shutting down vision node.")
        cv2.destroyAllWindows()   


def main(args):       

    try:
        ImageConverter()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down vision node.")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
