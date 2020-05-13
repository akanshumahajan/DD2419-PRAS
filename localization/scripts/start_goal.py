#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, TransformStamped
from crazyflie_driver.msg import Position
from binary_map_4n import path_planning_algo
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import json

class Startgoal:

    def __init__(self):
        
        self.start = start
        self.goal = goal

        ## Pull necessary ROS parameters from launch file:
        # Read goal message topic
        # param1 = rospy.search_param("start_message_topic") 
        # self.start_msg_topic = rospy.get_param(param1)       # /cf1/cmd_position
        self.start_msg_topic = '/planned_path'

        # Read path_planning message topic
        # param2 = rospy.search_param("goal_message_topic")
        # self.goal_msg_topic = rospy.get_param(param2)
        self.goal_msg_topic = '/final_goal'

        # Initialize callback variables
        #self.goal_msg = None
        # Initialize class variables
        self.goal_msg = None
        self.start_msg = None

        #self.path_pub = Path()

        # Establish subscription to control message
        #rospy.Subscriber(self.goal_msg_topic, Position, self.goal_msg_callback)
        # Delay briefly for subscriber to find message
        #rospy.sleep(2)

        # Establish publisher of converted Twist message
        self.pub_start = rospy.Publisher(self.start_msg_topic, PoseStamped, queue_size=10)
        self.pub_goal = rospy.Publisher(self.goal_msg_topic, PoseStamped, queue_size=10)

        # Initialize listener for estimated pose of vehicle in map frame
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)


    # def goal_msg_callback(self, goal_msg):
    #     self.goal_msg = goal_msg

    def start_position(self):
        
        self.start_msg = PoseStamped()
        self.start_msg.header.frame_id = 'map'
        trans = None
        target_frame = 'map'
        try:
            trans = tfBuffer.lookup_transform(target_frame,'cf1/odom',rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            #trans = self.tfBuffer.lookup_transform(self.map_frame, self.est_veh_pose_frame, rospy.Time(0), rospy.Duration(1.0))
            rospy.loginfo('Failure of lookup transfrom from  to map')
        if trans:
            self.start_msg.header.stamp = rospy.Time.now()
            self.start_msg.pose.position = trans.transform.translation
            self.start_msg.pose.orientation = trans.transform.rotation
            self.pub_start.publish(self.start_msg)

    def goal_position(self):

        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = 'map'

    def main(self, argv=sys.argv):
        
        rate = rospy.Rate(10)  # Hz

        while not rospy.is_shutdown():

            # Let ROS filter through the arguments
            args = rospy.myargv(argv=argv)

            if args!= None:
            # Load input world JSON
                with open(args[1], 'rb') as f:
                    self.world = json.load(f)
                
            else:  
            # Load default world JSON
                with open('~/dd2419_ws/src/courseackages/dd2419_resources/worlds_json/milestone3.world.json', 'rb') as f:
                    self.world = json.load(f)

        
            for m in self.world['markers']:
            #    transforms = [transform_from_marker(m)]
                self.marker_odom(m)
                #if transform!= None:

                    # Publish these transforms statically forever

            rate.sleep()



    def path_planning(self, start=(0, 0), end = (0, 0), algo = 'a-star', plot= False):
        
        rate = rospy.Rate(10) # 10hz
        
        while not rospy.is_shutdown():

            if self.goal_msg!= None:

                start = (self.start.x, self.start.y)
                
                # if self.goal_msg.x >= 0 and self.goal_msg.y>=0 :
                #     self.goal_msg.x = (10) - self.goal_msg.x
                #     self.goal_msg.y = (10) - self.goal_msg.y

                # elif self.goal_msg.x < 0 and self.goal_msg.y>=0 :    
                #     self.goal_msg.x = (10) - self.goal_msg.x
                #     self.goal_msg.y = (10) - self.goal_msg.y
                
                
                #     self.goal_msg.x = (10) - self.goal_msg.x
                #     self.goal_msg.y = (10) - self.goal_msg.y

                end= (self.goal_msg.x, self.goal_msg.y) # Converting into decimeter because map works in decimeter
                
                print ("start is :", start)
                print ("end is :", end)


                if algo == 'a-star':
                    path, path_px = path_planning_algo(start, end, algo, plot)
                    path_px = path_px 
                    print ("Path px is ",path_px)
                    #self.pub.publish("Sucess")
                    print ("Sucess!!!!!!!!!!!!!")
                else:
                    return 0

                self.path_planning_msg = PoseStamped()
                
                for i in range(len(path_px)):
                    self.x, self.y = path_px[i]
                    #print ('x is',type(self.x))
                    #print ('y is',self.y)                
                    self.path_planning_msg.header.frame_id = 'map'
                    self.path_planning_msg.header.stamp = rospy.Time.now()
                    #self.path_planning_msg.header.frame_id = 'map'


                    # self.path_planning_msg.pose.position.x = (self.x*(-0.1))+10 # # Converting into meter because rviz grid/ drone world works in meter
                    # self.path_planning_msg.pose.position.y = (self.y*(-0.1))+10 # # Converting into meter because rviz grid/ drone world works in meter
                    # self.path_planning_msg.pose.position.z = 0*(0.1) # # Converting into meter because rviz grid/ drone world works in meter
                    
                    # self.path_planning_msg.pose.orientation.x = 0*(0.1) # # Converting into meter because rviz grid/ drone world works in meter
                    # self.path_planning_msg.pose.orientation.y = 0*(0.1) # # Converting into meter because rviz grid/ drone world works in meter
                    # self.path_planning_msg.pose.orientation.z = 0*(0.1) # # Converting into meter because rviz grid/ drone world works in meter
                    # self.path_planning_msg.pose.orientation.w = 0*(0.1) # # Converting into meter because rviz grid/ drone world works in meter

                    self.path_planning_msg.pose.position.x = (self.x*(-0.01))+0.5 # # Converting into meter because rviz grid/ drone world works in meter; our world res is in cm and origin is 0.5 instead of 10
                    self.path_planning_msg.pose.position.y = (self.y*(-0.01))+0.5 # # Converting into meter because rviz grid/ drone world works in meter
                    self.path_planning_msg.pose.position.z = 0*(0.01) # # Converting into meter because rviz grid/ drone world works in meter
                    
                    self.path_planning_msg.pose.orientation.x = 0*(0.01) # # Converting into meter because rviz grid/ drone world works in meter
                    self.path_planning_msg.pose.orientation.y = 0*(0.01) # # Converting into meter because rviz grid/ drone world works in meter
                    self.path_planning_msg.pose.orientation.z = 0*(0.01) # # Converting into meter because rviz grid/ drone world works in meter
                    self.path_planning_msg.pose.orientation.w = 0*(0.01) # # Converting into meter because rviz grid/ drone world works in meter





                    #self.path.header = self.path_planning_msg.header
                    #self.path.poses.append(self.path_planning_msg)
                
                    #print(self.path)

                    self.pub.publish(self.path_planning_msg)
                    rate.sleep()



                #path_planning_msg = PoseStamped()

            else:
                rospy.spin()
            
        return path, path_px 



if __name__ == '__main__':

    # rospy.init_node('navgoal3', anonymous=True)
    # rospy.loginfo("Successful initilization of node")
    # tf_buf   = tf2_ros.Buffer()
    # tf_lstn  = tf2_ros.TransformListener(tf_buf)

    # cf = Crazy_flie()
    # cf.publish_cmd(0)
    # rospy.spin()

    rospy.init_node('Pathplanning', anonymous=True)
    rospy.loginfo("Successful initilization of node")


    quad = Pathplanning()
    quad.start_position()
    quad.path_planning(algo= 'a-star', plot = True)
    #rospy.spin()
    #rospy.init_node('navgoal3')
