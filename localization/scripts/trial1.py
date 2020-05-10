#!/usr/bin/env python
import time
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty

angle = 0
initialize = 0
mypose = None
state = 0
takeoff_cmd = 0
alt = 0
landing_cmd = 0
Rotate_cmd = 0

# Gives the hypotenuse distance from the current pose
def dist(myposex, myposey, goalx, goaly):
    dis = math.sqrt((myposex-goalx)**2+(myposey-goaly)**2)
    return dis

# Home position with some alt in odom frame
def Takeoff_cmd(tpose):
    global takeoff_cmd
    takeoff_cmd = Position()
    takeoff_cmd.header.frame_id = 'cf1/odom'
    takeoff_cmd.x = tpose.pose.position.x
    takeoff_cmd.y = tpose.pose.position.x
    takeoff_cmd.z = 0.4
    takeoff_cmd.yaw = 0
    return takeoff_cmd
    

# Landing function     
def Landing_callback(lpose):
    global alt, landing_cmd
    landing_cmd = Position()
    landing_cmd.header.frame_id = 'cf1/odom'
    landing_cmd.x = lpose.pose.position.x
    landing_cmd.y = lpose.pose.position.y
    alt = lpose.pose.position.z
    alt -= 0.1
    landing_cmd.z = alt
    landing_cmd.yaw = 0   
    if alt == 0 and lpose.pose.position.z == 0.000:
        #Landing done
        landing_cmd = None
    return landing_cmd


def Rotate_callback(rpose):
    global state, angle, Rotate_cmd

    Rotate_cmd = Position()
    Rotate_cmd.header.frame_id = 'cf1/odom'  

    angle += 0.5 
    Rotate_cmd.x = rpose.x
    Rotate_cmd.y = rpose.y
    Rotate_cmd.z = 0.4
    Rotate_cmd.yaw = angle

    if angle == 350:
        state = 3
        angle = 0

    return Rotate_cmd

# Define states according to the pose and return cmd0
def state_machine(cf1_pose):
    global state, mypose, initialize

    mypose = cf1_pose
    events = ['Takeoff','Checkpoint_clearance','Waypoint_nav',"land"]
    xval = round(mypose.pose.position.x,1)    
    yval = round(mypose.pose.position.y,1)
    zval = round(mypose.pose.position.z,1)
    thr = 0.1

    if initialize < 1 and zval < thr: #dist(xval, yval,0.0,0.0) < thr
        state = 1
        initialize += 1  #So that initialization happens only once
    
    if initialize == 1 and  zval == 0.4:
        state = 2
        initialize += 1

def publish_cmd(cmd):
    if cmd != None:
        cmd.header.stamp = rospy.Time.now()
        pub_cmd.publish(cmd)


rospy.init_node('Flight_Nav')
sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, state_machine)
pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)

def main():
    # rate = rospy.Rate(10)  # Hz
    while mypose == None: # Wait for 1st cf1/pose to be received
        continue
    while not rospy.is_shutdown():
        if state == 1:
            print (state)
            publish_cmd(Takeoff_cmd(mypose))
        elif state == 2:
            Rotate_callback(takeoff_cmd)
            publish_cmd(Rotate_cmd)
            print (state, angle)
        elif state == 3:
            # Landing_callback(mypose)
            # publish_cmd(landing_cmd)
            print (state)
        # elif state == 4:  
        # rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()