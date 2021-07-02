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

from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter:

    def __init__(self):

        self.node_name = "gem_record"
        
        rospy.init_node(self.node_name)
        
        rospy.on_shutdown(self.cleanup)
        
        # self.cv2_window_name = self.node_name
        # cv2.namedWindow(self.cv2_window_name, cv2.WINDOW_NORMAL)

        self.last_time = time.time()

        self.record_video = True
        
        # Create the cv_bridge object
        self.bridge    = CvBridge()

        self.image_sub = rospy.Subscriber("/mako_1/mako_1/image_raw", Image, self.image_callback)

        self.image_pub = rospy.Publisher("/front_camera/image_undisted", Image, queue_size=10)

        self.mtx       = np.array([[1046.17936,   0.0,          1011.65344],
                                   [0.0,          1051.07316,   737.852696],
                                   [0.0,          0.0,          1.0       ]])

        self.dist      = np.array([[-0.2331957, 0.09314091, -0.0004832, 0.00042556, -0.02042143]])

        self.M         = np.array([[-1.51720529e+00, -2.03584980e+00,  2.44534869e+03], \
                                   [ 6.36901052e-16, -3.44901217e+00,  2.40200225e+03], \
                                   [ 7.11683254e-19, -2.04631689e-03,  1.00000000e+00]])

        self.frame_width  = 1920 # 1520  # 1920
        self.frame_height = 1080 # 280  # 1080

        if (self.record_video):
            self.output_video = cv2.VideoWriter("/home/gem/record.avi", cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (self.frame_width, self.frame_height))
        
        # rospy.loginfo("Waiting for image topics...")


    def image_callback(self, ros_image):

        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # Important: frame is in BGR

        # Calculate frame rate
        fps_txt = "FPS: " + str(int(1/(time.time()-self.last_time)))
        
        # Process the frame using the process_image() function
        pub_image = self.process_image(frame, fps_txt)

        self.last_time = time.time()

        if (self.record_video):
            # img = pub_image[800:, 200:1720]
            img = cv2.cvtColor(pub_image, cv2.COLOR_BGR2RGB)
            # print(img.shape)
            self.output_video.write(img)
            rospy.loginfo("Recording ...")

        try:
            # Convert OpenCV image to ROS image and publish
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))                

    def process_image(self, frame, fps_txt):

    	img_undisted = cv2.undistort(frame, self.mtx, self.dist, None, self.mtx)

    	# cv2.putText(img_undisted, fps_txt, (60, 60), cv2.FONT_HERSHEY_PLAIN, 5, (255,0,0), 5)

    	return img_undisted
    
    
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
