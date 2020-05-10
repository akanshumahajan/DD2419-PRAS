#!/usr/bin/env python
import rospy
import numpy as np
import csv
from os.path import expanduser
# from nav_msgs.msg import Path
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
# import Waypoint_goals

# def odom_cb(data):
#     global pathxarray, patharray, pathyarray
#     patharray = []
#     pathxarray = []
#     pathyarray = []
#     x = 0
#     y = 0
#     pose = [x,y]  
#     for i in range(0,10):
#         pose[0] = round(data.pose.position.x,3)
#         pose[1] = round(data.pose.position.y,4)
#         patharray.append(pose)
#         pathxarray.append(pose[0])
#         pathyarray.append(pose[1])
#         if i == 9:
#             odom_sub.unregister()
#     xmean = round((np.mean(pathxarray)),4)
#     ymean = round((np.mean(pathyarray)),4)

def dict_goal():
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

rospy.init_node('Path_array')


# odom_sub = rospy.Subscriber('/cf1/pose', PoseStamped, odom_cb)
# path_pub = rospy.Publisher('/path', Path, queue_size=10)
def main():
    dict_goal()
    # rospy.spin()

if __name__ == '__main__':
    main()
    # if i > 9:
    #     rospy.on_shutdown()
    # rospy.spin()
