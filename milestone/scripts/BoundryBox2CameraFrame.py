#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 18 15:29:30 2020

@author: Fredrik Forsberg
"""

from .Undistort import Undistort
import rospy
import numpy as np
from geometry_msgs.msg import PointStamped, PolygonStamped

###


class BoundryBox2CameraFrame:
    def __init__(self, node_name, camera_info_subscription_topic, camera_pose_publishing_topic, boundry_box_):
        # Initiate node
        rospy.init_node(node_name)
        
        # Class to keep track of the camera info
        self.undistort = Undistort(camera_info_subscription_topic)
        
        # Create a publisher to send the position in the camera frame
        self.camera_pose_publisher = rospy.Publisher(camera_pose_publishing_topic, PointStamped, queue_size=2)
        # Subscribe to get object boundry box
        self.boundry_box_subscriber = rospy.Subscriber(raw_image_subscription_topic, Image, self.raw_image_callback)
    
###
        
    
if __name__ == '__main__':
    image2camera = BoundryBox2CameraFrame('ImageFrame2CameraFrame', '/cf1/camera/camera_info')