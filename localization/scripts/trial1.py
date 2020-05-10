#!/usr/bin/env python
import time
import math
from os.path import expanduser
import numpy as np
import rospy
import csv
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
import Waypoint_goals


angle = 0
initialize = 0
thr = 0.1
mypose = None
dictpath = []



def init_pose(data):
    global xmean, ymean
    patharray = []
    pathxarray = []
    pathyarray = []
    x = 0
    y = 0
    pose = [x,y]  
    for i in range(0,10):
        pose[0] = round(data.pose.position.x,3)
        pose[1] = round(data.pose.position.y,4)
        patharray.append(pose)
        pathxarray.append(pose[0])
        pathyarray.append(pose[1])
        if i == 9:
            initial_pose.unregister()
    xmean = round((np.mean(pathxarray)),4)
    ymean = round((np.mean(pathyarray)),4)
    print xmean, ymean

def dict_goal():
    global dictgoal
    dictgoal = []
    pathcsv = expanduser('~')
    pathcsv += '/dd2419_ws/src/DD2419-PRAS/localization/scripts/Pathcsv'
    with open(pathcsv, 'rb') as csv_file:
        csv_reader = csv.reader(csv_file)
        next(csv_reader)
        for line in csv_reader:
            line[0] = float(line[0])
            line[1] = float(line[1])
            dictgoal.append(line)
    print dictgoal
    return dictgoal

# Gives the hypotenuse distance from the current pose
def dist(myposex, myposey, goalx, goaly):
    dis = math.sqrt((myposex-goalx)**2+(myposey-goaly)**2)
    return dis

# Gives current pose in odom frame
# def current_pose(msg):
#     global cu_pose
#     cu_pose = msg
    

# def pose_to_yaw(pose):
#     global yaw
#     roll,pitch,yaw = euler_from_quaternion (pose.pose.orientation.x,
#     pose.pose.orientation.y, pose.pose.orientationz, pose.pose.orientation.w)
#     roll = math.degrees(round(roll,2))
#     pitch = math.degrees(round(pitch,2))
#     yaw = math.degrees(round(yaw,2)),

# def Path_cb(data):
#     a = 0
#     b = 0
#     pose = [a,b]
#     pose[0] = data.pose.position.x
#     pose[1] = data.pose.position.y
#     dictpath.append(pose)

# Home position with some alt in odom frame
def Takeoff_cmd(xmean,ymean):
    global takeoff_cmd
    takeoff_cmd = Position()
    takeoff_cmd.header.frame_id = 'cf1/odom'
    takeoff_cmd.x = xmean
    takeoff_cmd.y = ymean
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


# def Rotate_callback(rpose):
#     global state, angle, Rotate_cmd

#     Rotate_cmd = Position()
#     Rotate_cmd.header.frame_id = 'cf1/odom'  

#     angle += 0.5 
#     Rotate_cmd.x = rpose.x
#     Rotate_cmd.y = rpose.y
#     Rotate_cmd.z = 0.4
#     Rotate_cmd.yaw = angle

#     if angle == 360:
#         angle = 0
        # state = 3

    # return Rotate_cmd

# def StaticPos_cmd(spose):
#     global Spose_cmd
#     Spose_cmd = Position()
#     Spose_cmd.header.frame_id = 'cf1/odom'
#     Spose_cmd.x = round(spose.pose.position.x,2)
#     Spose_cmd.y = round(spose.pose.position.y,2)
#     Spose_cmd.z = 0.4
#     Spose_cmd.yaw = angle
#     return Spose_cmd

# Define states according to the pose and return cmd0
def state_machine(cf1_pose):
    global  mypose, initialize, state, rval

    mypose = cf1_pose
    xval = round(mypose.pose.position.x,2)    
    yval = round(mypose.pose.position.y,2)
    zval = round(mypose.pose.position.z,2)
    thr = 0.1
    rval = Position()

    if initialize < 1 and zval < thr: #dist(xval, yval,0.0,0.0) < thr
        state = 1
        initialize += 1  #So that initialization happens only once
    
    if initialize == 1 and  (zval- 0.40) < thr:
        # state = 2
        state = 3
        initialize += 1 

    # if state == 2 and aruco = detect and zval == 0.4:
    #     StaticPos_cmd(mypose)
    #     state = 3

    # if state == 3 and transform available and zval == 0.4:
    #     state = 4
    #     execute path in goalpub(cmd1)
    for ii in dictgoal:   # for one waypoint navigation 
        xgoal = dictgoal[ii][0]
        ygoal = dictgoal[ii][1]
        if dist(xval,yval,xgoal,ygoal) > thr:  #Navigate path  
            state = 3
        elif dist(xval,yval,xgoal,ygoal) < thr: # Reached Waypoint goal rotate callback
            rval.x = xval
            rval.y = yval            
            state = 4
        elif ii == (len(dictgoal)-1) and dist(xval,yval,xgoal,ygoal) < thr:
            state = 5


    # elif dist(xval,yval,takeoff_cmd.x,takeoff_cmd.y) < thr: # Back to home landing callback           
    #         state = 5

    # if initialize > 2 and zval == 0 and alt == 0:
    #     state = 6


def goalpub(cmdx,cmdy):
    global odom_cmd

    # rate = rospy.Rate(10)
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    goal.pose.position.x = cmdx
    goal.pose.position.y = cmdy
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
        cmd.yaw = 180
        odom_cmd = cmd
        return odom_cmd

def publish_cmd(cmd):
    if cmd != None:
        cmd.header.stamp = rospy.Time.now()
        pub_cmd.publish(cmd)


rospy.init_node('Flight_Nav')
sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, state_machine)
initial_pose = rospy.Subscriber('/cf1/pose', PoseStamped, init_pose)
pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
tf_buf = tf2_ros.Buffer()
tf_lstn = tf2_ros.TransformListener(tf_buf)

def main():
    dict_goal()
    rate = rospy.Rate(10)  # Hz
    while mypose == None: # Wait for 1st cf1/pose to be received
        continue
    while not rospy.is_shutdown():
        if state == 1:
            print state
            Takeoff_cmd(xmean,ymean)
            publish_cmd(takeoff_cmd)
            print state
        # elif state == 2:
        #     Rotate_callback(takeoff_cmd)
        #     publish_cmd(Rotate_cmd)
        #     print state, angle
        elif state == 3:
            for e in dictgoal:
                xcmd = dictgoal[e][0]
                ycmd = dictgoal[e][1]
                goalpub(xcmd,ycmd)
                publish_cmd(odom_cmd)
            # print state
        # elif state == 4:
        #     Rotate_callback(rval)
        #     publish_cmd(Rotate_cmd)
        #     print state, angle
        elif state == 5:
            Landing_callback(mypose)
            publish_cmd(landing_cmd)
        elif state == 6:
            publish_cmd(None)
        rate.sleep()
    # rospy.spin()

if __name__ == '__main__':
    main()