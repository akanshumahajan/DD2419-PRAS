#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 18 15:29:30 2020

@author: Fredrik Forsberg
"""

from .Undistort import Undistort
import rospy
import numpy as np
from geometry_msgs.msg import PointStamped, PolygonStamped  # TODO Not handeling identifiers for the signs and multiple

###


class BoundryBox2CameraFrame:
    def __init__(self, node_name, camera_info_subscription_topic, camera_pose_publishing_topic, 
                 boundry_box_subscription_topic):
        # Initiate node
        rospy.init_node(node_name)
        
        # Class to keep track of the camera matices and image dimensions
        self.camera_properties = Undistort(camera_info_subscription_topic)
        
        # Create a publisher to send the position in the camera frame
        self.camera_pose_publisher = rospy.Publisher(camera_pose_publishing_topic, PointStamped, queue_size=2)
        # Subscribe to get object boundry box
        self.boundry_box_subscriber = rospy.Subscriber(boundry_box_subscription_topic, PolygonStamped, 
                                                       self.boundry_box_2_camera_frame)
    
    
    def boundry_box_2_camera_frame(self, boundry_box):
        if len(boundry_box.Polygon) != 5:
            print('Wrong polygon length. Expected 5, but got %d.' % len(boundry_box.Polygon))
            return
        
        # self.camera_properties.calibration_camera_matrix = array([[245.15986633,   0.        , 319.99919435],
        #                                                           [  0.        , 245.03199768, 239.99895306],
        #                                                           [  0.        ,   0.        ,   1.        ]])
        
        # Image center
        image_center = np.array([self.camera_properties.calibration_camera_matrix[0][2]], 
                                 self.camera_properties.calibration_camera_matrix[1][2], 
                                 0.0], dtype=np.float64)
        
        # Focal length (a mean since we have two almost identical)
        f = (self.camera_properties.calibration_camera_matrix[0][0] + 
             self.camera_properties.calibration_camera_matrix[1][1]) / 2.0
        
        # The points of the boundry box
        polygon_points = [np.asarray([p.x, p.y, f], dtype=np.float64) + image_center for p in boundry_box.Polygon[:-1]]
        
        # Cet the inverse camera matrix
        inverse_calibration_matrix = np.linalg.inv(self.camera_properties.calibration_camera_matrix)
        
        # Transform the points
        transformed_points = [inverse_calibration_matrix.dot(p) for p in polygon_points]
        
        # Center
        center = np.mean(np.array(transformed_points), axis=0)
        
        # Height
        # Finding the distance between the pair of "most vertical" connected points
        positive_corner_vectors = [np.abs(transformed_points[(i+1)%4] - transformed_points[i%4]) for i in range(0, 4)]
        verticality = [np.array([0, 1, 0]).dot(v/np.linalg.norm(v)) for v in positive_corner_vectors]
        height = verticality.index(max(verticality))
        
        # TODO Compare the height 
        
        obj_point = PointStamped()
        obj_point.header.frame_id = 'cf1/camera_link'
        obj_point.header.stamp = boundry_box.header.stamp
        (obj_point.Point.x,
         obj_point.Point.y,
         obj_point.Point.z) = depth_point
        # Publish
        self.camera_pose_publisher.publish(obj_point)
    
###
        
    
if __name__ == '__main__':
    image2camera = BoundryBox2CameraFrame('ImageFrame2CameraFrame', '/cf1/camera/camera_info', '/dd2419/sign_point', 
                                          '/dd2419/sign_boundry_box')