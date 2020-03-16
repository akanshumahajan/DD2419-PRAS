#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 25 11:25:09 2020

@author: Fredrik Forsberg
"""

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


class PoseInfoClass:
    def __init__(self, pose_subscription_topic='/cf1/pose'):
        self._pose = PoseStamped()
        self.zero_timestamp = True
        self.pose_listener = rospy.Subscriber(pose_subscription_topic, PoseStamped, self.pose_callback)
        
        
    def pose_callback(self, pose):
        self._pose = pose
        
        if self.zero_timestamp and self._pose.header.stamp.to_sec() > 0:
            self.zero_timestamp = False
        
        
    def get_pose(self, timeout=2.5):
        if not self.zero_timestamp:
            return self._pose
        else:
            # No pose has been recieved yet. Waiting...
            import time
            sleep_time = 0.001
            for i in range(0, int(timeout / sleep_time)):
                if not self.zero_timestamp:
                    return self._pose
                time.sleep(sleep_time)
            raise Exception('Timeout for getting a pose with non-zero timestamp in PoseInfoClass')
        
        
    def get_angles(self):
        # Roll, Pitch, Yaw
        p = self.get_pose()
        return euler_from_quaternion((p.pose.orientation.x,
                                      p.pose.orientation.y,
                                      p.pose.orientation.z,
                                      p.pose.orientation.w))
        
        
    def get_position(self):
        p = self.get_pose()
        return p.pose.position.x, p.pose.position.y, p.pose.position.z
        