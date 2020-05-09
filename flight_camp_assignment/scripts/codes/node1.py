#!/usr/bin/env python

import sys
import math
import json

import rospy
import tf2_ros 
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Vector3
from aruco_msgs.msg import MarkerArray

global goal = None


def goal_callback(msg):
    goal = msg

    
def publish_marker(i):
    global broadcaster

    for i in goal.markers:
        marker_t = PostStamped()
        marker_t.pose = m.pose.pose
        marker_t.header.frame_id = 'camera_link'
        marker_t.header.stamp = rospy.Time.now()

    # Check if tranform is avaliable 
    if not tf_buf.can_transform(marker.header.frame_id, 'map', marker.header.stamp):
        rospy.logwarn_throttle(5.0, 'No transform from %s to map' % marker.header.frame_id)
        return

    marker_map = tf_buf.transform (marker_t, 'map')

    marktf = TransformStamped()
    marktf.header.frame_id = 'map'
    marktf.child_frame_id = 'aruco/detected' + str(m.id)
    marktf.transform.translation = marker_map.pose.position
    marktf.transform.rotation = marker_map.pose.orientation
    
    broadcaster.sendTransform(marktf)
    #return marktf

 # Publish these transforms statically forever
    rospy.init_node('node1')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    sub_goal = rospy.Subscriber('/aruco/markers', MarkerArray, goal_callback)
    tf_buf   = tf2_ros.Buffer()
    tf_lstn  = tf2_ros.TransformListener(tf_buf)
    rospy.spin()


def main():
    #rate = rospy.Rate(10)  # Hz
    while not rospy.is_shutdown():
        if goal:
            publish_marker(i)
        #rate.sleep()

if __name__ == "__main__":
    main()