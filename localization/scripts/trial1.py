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


def dist(myposex, myposey, markx, marky):
    dis = math.sqrt((myposex-markx)**2+(myposey-marky)**2)
    return dis


def new_pose(mypose):
    global origin, state, lap, initalize
    if initalize < 1:
        origin = get_origin(mypose)
        initalize += 1
    tol = 0.1
    if dist(mypose.pose.position.x, mypose.pose.position.y, origin.x, origin.y) < tol and (origin.z-mypose.pose.position.z) < tol and lap != N:
        state = 1
        lap = 0
    elif dist(mypose.pose.position.x, mypose.pose.position.y, setpoint.x, setpoint.y) < tol and lap == 0:
        state = 2

def goalpub(x,y,yaww):
    global odom_cmd
    rate = rospy.Rate(10)
    goal = PoseStamped()
    goal.header.stamp = stamp
    goal.header.frame_id = "map"
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.4
    if not tf_buf.can_transform('map', 'cf1/odom', rospy.Time.now(), timeout=rospy.Duration(0.2)):
        rospy.logwarn_throttle(10.0, 'No transform from %s to cf1/odom' % goal.header.frame_id)
        return

    goal_odom = tf_buf.transform(goal, 'cf1/odom')

    if goal_odom:
        #print(goal_odom.header.stamp)
        cmd = Position()
        cmd.header.stamp = rospy.Time.now()
        cmd.x = goal_odom.pose.position.x
        cmd.y = goal_odom.pose.position.y
        cmd.z = goal_odom.pose.position.z
        cmd.yaw = yaww
        odom_cmd = cmd
        return odom_cmd




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