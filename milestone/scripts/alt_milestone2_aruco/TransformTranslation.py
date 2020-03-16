#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 26 13:20:13 2020

@author: Fredrik Forsberg
"""

import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs  # Needed
from geometry_msgs.msg import TransformStamped, PoseStamped

###


class TransformTranslation:
    def __init__(self, node_name, aruco_names, sign_names, stabilisation_method='mean'):
        # Create node
        rospy.init_node(node_name)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        
        self.aruco_names = aruco_names
        self.sign_names = sign_names
        
        self.map_to_odom = None
        
        self.stabilisation_method = stabilisation_method
        
        # Variables used while calculating a mean transform
        self.last_aruco_timestamp = 0
        self.last_sign_timestamp = 0
        self.aruco_transforms = []
        self.sign_transforms = []
        # Hard-coded parameters
        self.delta_time = 1.  # Time ago in seconds for which transforms are allowed to be a part of the mean
        self.detect_rate = 10  # Hz for the nodes ArUcoDetect and SignDetect
        self.add_mean_limit = 1 / 3.  # When only this part of the expected entries, start adding last means
        
        
    def run(self):
        rate = rospy.Rate(10)  # Hz
        
        while not rospy.is_shutdown():
            
            # Get transforms from detected ArUco markers and signs
            new_aruco_transforms = self.get_aruco_transforms()
            new_sign_transforms = self.get_sign_transforms()
            
            # TODO Kalman filter instead of, or including, the following mean value
            
            if self.stabilisation_method == 'mean':
                # Add the new transforms to lists of older ones
                self.aruco_transforms += new_aruco_transforms
                self.sign_transforms += new_sign_transforms
                
                # Remove too old transforms
                self.aruco_transforms = self.remove_old_transforms(self.aruco_transforms, self.delta_time)
                self.sign_transforms = self.remove_old_transforms(self.sign_transforms, self.delta_time)
                
                # If too few transforms, add the previous mean value a set number of times
                mean_transform_list = []
                added_mean_values = self.heaviside(self.detect_rate * self.delta_time * self.add_mean_limit - 
                                                   len(self.aruco_transforms) - len(self.sign_transforms))
                if (self.map_to_odom is not None) and (added_mean_values > 0.):
                    mean_transform_list = [self.map_to_odom] * int(np.ceil(added_mean_values))
                
                # Calculate the mean transform with wieghts for how trusted the readings are
                mean_transform = self.transform_mean(self.aruco_transforms, self.sign_transforms, 
                                                     mean_transform_list, 
                                                     aruco_weight=3, sign_weight=1, mean_weight=2)
                
                self.map_to_odom = mean_transform
                
                
            # TODO Trigger relocalisation mode if the self.map_to_odom is getting too old and too unreliable
            
            # Broadcast the transform
            if self.map_to_odom is not None:
                self.tf_broadcaster.sendTransform(self.map_to_odom)
            
            rate.sleep()
        
        # rospy.spin() shouldn't be needed since it sleeps until rospy.is_shutdown() returns True
        # Keeping it to be on the safe side
        rospy.spin()
        
        
    def get_aruco_transforms(self):
        detected_aruco = []
            
        for aruco_name in self.aruco_names:
            if self.tf_buf.can_transform("cf1/odom", aruco_name, rospy.Time(0.)):
                detected_aruco.append(self.tf_buf.lookup_transform(aruco_name, "cf1/odom", rospy.Time(0.)))
                
        timestamps = [t.header.stamp.to_sec() for t in detected_aruco]
        
        detected_aruco = [detected_aruco[i] for i in range(0, len(detected_aruco)) if 
                          (timestamps[i] > self.last_aruco_timestamp)]
    
        if len(detected_aruco) > 0:
            self.last_aruco_timestamp = max(timestamps)
        
        aruco_map_to_odom = []
        
        for t in detected_aruco:
            aruco_marker_name = ("aruco/marker" + t.header.frame_id.replace("aruco/detected", ""))
            # Link the ArUco marker in the map frame and the odom frame
            marker_to_odom_pose = PoseStamped()
            marker_to_odom_pose.header.frame_id = aruco_marker_name
            marker_to_odom_pose.header.stamp = t.header.stamp
            marker_to_odom_pose.pose.position = t.transform.translation
            marker_to_odom_pose.pose.orientation = t.transform.rotation
            
            # Test if it is possible to transform from map to the ArUco marker
            if self.tf_buf.can_transform("map", aruco_marker_name, marker_to_odom_pose.header.stamp):
                
                map_to_odom_pose = self.tf_buf.transform(marker_to_odom_pose, "map")
                
                transform = TransformStamped()
                transform.header.stamp = marker_to_odom_pose.header.stamp
                transform.header.frame_id = "map"
                transform.child_frame_id = "cf1/odom"
            
                transform.transform.translation = map_to_odom_pose.pose.position
                transform.transform.rotation = map_to_odom_pose.pose.orientation
                
                aruco_map_to_odom.append(transform)
            
            else:
                rospy.logwarn_throttle(5.0, 'No transform from %s to map' % marker_to_odom_pose.header.frame_id)
        
        return aruco_map_to_odom
    
    
    def get_sign_transforms(self):
        detected_sign = []
            
        for sign_name in self.sign_names:
            if self.tf_buf.can_transform("cf1/odom", sign_name, rospy.Time(0.)):
                detected_sign.append(self.tf_buf.lookup_transform(sign_name, "cf1/odom", rospy.Time(0.)))
                
        timestamps = [t.header.stamp.to_sec() for t in detected_sign]
        
        detected_sign = [detected_sign[i] for i in range(0, len(detected_sign)) if 
                         (timestamps[i] > self.last_sign_timestamp)]
        
        if len(detected_sign) > 0:
            self.last_sign_timestamp = max(timestamps)
        
        sign_map_to_odom = []
        
        for t in detected_sign:
            sign_marker_name = ('roadsign/sign_' + t.header.frame_id.replace('roadsign/detected_', ""))
            # Link the sign in the map frame and the odom frame
            sign_to_odom_pose = PoseStamped()
            sign_to_odom_pose.header.frame_id = sign_marker_name
            sign_to_odom_pose.pose.position = t.transform.translation
            sign_to_odom_pose.pose.orientation = t.transform.rotation
            
            # Test if it is possible to transform from map to the ArUco marker
            if self.tf_buf.can_transform("map", sign_marker_name, sign_map_to_odom.header.stamp):  # rospy.Time(0.)):
                
                map_to_odom_pose = self.tf_buf.transform(sign_to_odom_pose, "map")
                
                transform = TransformStamped()
                transform.header.stamp = t.header.stamp
                transform.header.frame_id = "map"
                transform.child_frame_id = "cf1/odom"
            
                transform.transform.translation = map_to_odom_pose.pose.position
                transform.transform.rotation = map_to_odom_pose.pose.orientation
                
                sign_map_to_odom.append(transform)
            
            else:
                rospy.logwarn_throttle(5.0, 'No transform from %s to map' % marker_to_odom_pose.header.frame_id)
        
        return sign_map_to_odom
    
    
    def remove_old_transforms(self, transforms, delta_time=1.):
        time_now = rospy.Time.now().to_sec()
        return [t for t in transforms if ((time_now - t.header.stamp.to_sec()) <= delta_time)]
    
    
    @staticmethod
    def heaviside(value):
        if value >= 0.:
            return value
        else:
            return 0.
        
        
    def transform_mean(self, aruco_transforms, sign_transforms, mean_transforms, 
                       aruco_weight=3, sign_weight=1, mean_weight=2):
        
        transforms_list = aruco_transforms + sign_transforms + mean_transforms
        
        if not transforms_list:
            return None
                
        # Create wights associated with how trusted the readings are
        weights = np.ones(((len(self.aruco_transforms) + len(self.sign_transforms) + len(mean_transforms)),))
        weights[0:len(self.aruco_transforms)] = aruco_weight
        weights[len(self.aruco_transforms):len(self.aruco_transforms)+len(self.sign_transforms)] = sign_weight
        weights[len(self.aruco_transforms)+len(self.sign_transforms):] = mean_weight
        
        positions = np.zeros((len(transforms_list), 3), dtype=np.float64)
        quaternions = np.zeros((len(transforms_list), 4), dtype=np.float64)
        
        for i in range(0, len(transforms_list)):
            positions[i, 0] = transforms_list[i].transform.translation.x
            positions[i, 1] = transforms_list[i].transform.translation.y
            positions[i, 2] = transforms_list[i].transform.translation.z
            quaternions[i, 0] = transforms_list[i].transform.rotation.x
            quaternions[i, 1] = transforms_list[i].transform.rotation.y
            quaternions[i, 2] = transforms_list[i].transform.rotation.z
            quaternions[i, 3] = transforms_list[i].transform.rotation.w
        
        mean_position = np.sum((weights * positions.T).T, axis=0) / np.sum(weights)
        
        # Quaternion mean from scipy.spatial.transform.Rotation which is only availiable for python 3+ and not for 2.7
        K = np.dot(weights * quaternions.T, quaternions)
        l, v = np.linalg.eigh(K)
        mean_rotation = v[:, -1] / np.linalg.norm(v[:, -1])
        
        mean_transform = TransformStamped()
        mean_transform.header.stamp = rospy.Time.from_sec(max([self.last_aruco_timestamp, self.last_sign_timestamp]))
        mean_transform.header.frame_id = "map"
        mean_transform.child_frame_id = "cf1/odom"
        mean_transform.transform.translation.x = mean_position[0]
        mean_transform.transform.translation.y = mean_position[1]
        mean_transform.transform.translation.z = mean_position[2]
        mean_transform.transform.rotation.x = mean_rotation[0]
        mean_transform.transform.rotation.y = mean_rotation[1]
        mean_transform.transform.rotation.z = mean_rotation[2]
        mean_transform.transform.rotation.w = mean_rotation[3]
        
        return mean_transform
    
###
        
    
if __name__ == '__main__':
    aruco_names = ["aruco/detected" + str(i) for i in range(0, 16)]
    signs = ['airport', 'dangerous_curve_left', 'dangerous_curve_right', 'follow_left', 'follow_right', 'junction',
                  'no_bicycle', 'no_heavy_truck', 'no_parking', 'no_stopping_and_parking', 'residential', 
                  'road_narrows_from_left', 'road_narrows_from_right', 'roundabout_warning', 'stop']
    # sign_names = ['roadsign/detected_' + sign for sign in signs]
    sign_names = ['roadsign/detected_' + sign + '_' + str(i) for sign in signs for i in range(0, 2)]
    
    transform_translation = TransformTranslation('TransformTranslation', aruco_names, sign_names)
    
    transform_translation.run()
