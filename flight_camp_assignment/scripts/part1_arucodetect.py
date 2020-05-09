#!/usr/bin/env python

import sys
import math
import json
import tf_conversions
import tf2_geometry_msgs
import rospy
import tf2_ros 
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped
from aruco_msgs.msg import MarkerArray

aruco = None

def aruco_pose(aruco_array):
    global aruco
    aruco= []
    transform = TransformStamped()
    for m in range(len(aruco_array.markers)):
        transform.header.frame_id = 'cf1/camera_link'
        transform.child_frame_id = 'aruco/detected' + str(aruco_array.markers[m].id)
        transform.transform.translation = aruco_array.markers[m].pose.pose.position
        transform.transform.rotation = aruco_array.markers[m].pose.pose.orientation
        aruco.append(transform)

def publish_aruco_tf(aruco):
    if aruco!= None:
        for m in range (len(aruco)):
            aruco[m].header.stamp = rospy.Time.now()
            tf_bc.sendTransform(aruco[m])
        aruco = None


def main():
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        if aruco:
            publish_aruco_tf(aruco)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('part1_arucodetect')
    aruco_estimate = rospy.Subscriber('/aruco/markers',MarkerArray,aruco_pose, queue_size=1)
    tf_bc = tf2_ros.TransformBroadcaster()
    main()