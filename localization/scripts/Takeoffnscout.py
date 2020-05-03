#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty

state = 0
angle = 0
# Current pose (global state)
#pose = None


""" takeoff_cmd = Position()
takeoff_cmd.header.frame_id = 'cf1/odom'
takeoff_cmd.x = 0
takeoff_cmd.y = 0
takeoff_cmd.z = 0.4


Rotate_cmd = Position()
Rotate_cmd.header.frame_id = 'cf1/odom'   
Rotate_cmd.x = 0
Rotate_cmd.y = 0
Rotate_cmd.z = 0.5
Rotate_cmd.yaw = 0 """

def origin(msg):
    global origin

    origin = Position()
    origin.header.frame_id = 'cf1/odom'
    origin.x = msg.pose.position.x
    origin.y = msg.pose.position.y
    origin.z = 0.4
    origin.yaw = 0
    origin = pose
    return origin

def Rotate_callback(pose):

    global state, angle, Rotate_cmd

    Rotate_cmd = Position()
    Rotate_cmd.header.frame_id = 'cf1/odom'  

    angle += 2 
    Rotate_cmd.x = pose.pose.position.x
    Rotate_cmd.y = pose.pose.position.y
    Rotate_cmd.z = 0.5
    Rotate_cmd.yaw = angle
    rospy.loginfo('Rotating now angle:\n%s',Rotate_cmd.yaw)
    pub_cmd.publish(Rotate_cmd)
    
    if angle == 360:
        angle = 0
        state = 3
        
        
    if pose.pose.position.x == 0 and pose.pose.position.y == 0 and pose.pose.position.z == 0:
        state = 1
        print("State1")

    if pose.pose.position.x == 0 and pose.pose.position.y == 0 and pose.pose.position.z == 0.6:
        state = 2
        print("State2")

rospy.init_node('Takeoff_n_Scout')
sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, Rotate_callback)
pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)

def main():
    rate = rospy.Rate(10)  # Hz
    while not rospy.is_shutdown():
        if state == 1:
            pub_cmd.publish(origin)
        elif state == 2:
            pub_cmd.publish(Rotate_cmd)
        elif state == 3:
            rospy.is_shutdown()

        rate.sleep()

if __name__ == '__main__':
    main()