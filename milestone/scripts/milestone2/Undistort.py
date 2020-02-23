#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 18 13:33:31 2020

@author: Fredrik Forsberg
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

###


class Undistort(object):
    def __init__(self, camera_info_subscription_topic):
        self.image_height = None
        self.image_width = None
        self.camera_matrix = np.identity(3, dtype=np.float64)
        self.distortion = np.ones((1, 5), dtype=np.float64)
        self.calibration_camera_matrix = np.identity(3, dtype=np.float64)
        self.roi = (0, 0, 0, 0)
        
        # Camera info subscription
        rospy.Subscriber(camera_info_subscription_topic, CameraInfo, self.camera_info_callback)
    
    
    def camera_info_callback(self, camera_info):
        self.camera_matrix = np.asarray(camera_info.K, dtype=np.float64).reshape((3, 3))
        self.distortion = np.asarray(camera_info.D, dtype=np.float64)
        self.image_height = camera_info.height
        self.image_width = camera_info.width
        self.calibration_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.distortion, 
                                                                            (self.image_width, self.image_height), 1, 
                                                                            (self.image_width, self.image_height))
        
    
    def undistort(self, img):
        if None in [self.image_height, self.image_width]:
            return None
        
        dst = cv2.undistort(img, self.camera_matrix, self.distortion, None, self.calibration_camera_matrix)
        x, y, w, h = self.roi
        
        return dst[y:y+h, x:x+w]
        
#
        
        
class UndistortNode(Undistort):
    def __init__(self, node_name, camera_info_subscription_topic, raw_image_subscription_topic, 
                 undistorted_image_publishing_topic):
        # Initiate node
        rospy.init_node(node_name)
        
        # Inherit from Undistort
        super(UndistortNode, self).__init__(camera_info_subscription_topic)
        
        # Create bridge between ROS and OpenCV
        self.bridge = CvBridge()
        # Camera feed subscription
        self.raw_image_subscriber = rospy.Subscriber(raw_image_subscription_topic, Image, self.raw_image_callback)
        
        # Undistorted publisher
        self.undistorted_publisher = rospy.Publisher(undistorted_image_publishing_topic, Image, queue_size=2)
        
        # Keep python from exiting
        try:
            rospy.spin()
        except KeyboardInterrupt as err:
            print(repr(err))
            
        
    def raw_image_callback(self, ros_image):
        try:
            img = self.bridge.imgmsg_to_cv2(ros_image, 'bgr8')
        except CvBridgeError as err:
            print(repr(err))
            return
        
        undist_img = self.undistort(img)
        
        if undist_img is not None:
            try:
                self.undistorted_publisher.publish(self.bridge.cv2_to_imgmsg(undist_img, "bgr8"))
            except CvBridgeError as err:
                print(repr(err))
        
    
###
        
    
if __name__ == '__main__':
    undistort_node = UndistortNode('ImageFrame2CameraFrame', '/cf1/camera/camera_info', '/cf1/camera/image_raw', 
                                   '/undistorted_image')
