#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, TransformStamped
from crazyflie_driver.msg import Position
from binary_map_4n import localization_algo
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class Localization:

    def __init__(self, start= None, goal = None, algo = 'a-star'):
        
        self.start = start
        self.goal = goal

        ## Pull necessary ROS parameters from launch file:
        # Read goal message topic
        # param1 = rospy.search_param("goal_message_topic") 
        # self.goal_msg_topic = rospy.get_param(param1)       # /cf1/cmd_position
        self.goal_msg_topic = '/final_goal'

        # Read Localization message topic
        # param2 = rospy.search_param("localization_message_topic")
        # self.localization_msg_topic = rospy.get_param(param2)
        self.localization_msg_topic = '/planned_path'

        # Initialize callback variables
        self.goal_msg = None
        # Initialize class variables
        self.localization_msg = None

        self.path_pub = Path()

        # Establish subscription to control message
        rospy.Subscriber(self.goal_msg_topic, Position, self.goal_msg_callback)
        # Delay briefly for subscriber to find message
        rospy.sleep(2)

        # Establish publisher of converted Twist message
        self.pub = rospy.Publisher(self.localization_msg_topic, PoseStamped, queue_size=10)

    def goal_msg_callback(self, goal_msg):
        self.goal_msg = goal_msg

    def start_position(self):
        self.start = Position()
        self.start.x = 10 # Converting into decimeter because map works in decimeter
        self.start.y = 10 # Converting into decimeter because map works in decimeter
        self.start.z = 0
        self.start.yaw = 0

    def localize (self, start=(0, 0), end = (0, 0), algo = 'a-star', plot= False):
        
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
                    path, path_px = localization_algo(start, end, algo, plot)
                    path_px = path_px 
                    print ("Path px is ",path_px)
                    #self.pub.publish("Sucess")
                    print ("Sucess!!!!!!!!!!!!!")
                else:
                    return 0

                self.localization_msg = PoseStamped()
                
                for i in range(len(path_px)):
                    self.x, self.y = path_px[i]
                    #print ('x is',type(self.x))
                    #print ('y is',self.y)                
                    self.localization_msg.header.frame_id = 'map'
                    self.localization_msg.header.stamp = rospy.Time.now()
                    #self.localization_msg.header.frame_id = 'map'


                    self.localization_msg.pose.position.x = (self.x*(-0.1))+10 # # Converting into meter because rviz grid/ drone world works in meter
                    self.localization_msg.pose.position.y = (self.y*(-0.1))+10 # # Converting into meter because rviz grid/ drone world works in meter
                    self.localization_msg.pose.position.z = 0*(0.1) # # Converting into meter because rviz grid/ drone world works in meter
                    
                    self.localization_msg.pose.orientation.x = 0*(0.1) # # Converting into meter because rviz grid/ drone world works in meter
                    self.localization_msg.pose.orientation.y = 0*(0.1) # # Converting into meter because rviz grid/ drone world works in meter
                    self.localization_msg.pose.orientation.z = 0*(0.1) # # Converting into meter because rviz grid/ drone world works in meter
                    self.localization_msg.pose.orientation.w = 0*(0.1) # # Converting into meter because rviz grid/ drone world works in meter

                    #self.path.header = self.localization_msg.header
                    #self.path.poses.append(self.localization_msg)
                
                    #print(self.path)

                    self.pub.publish(self.localization_msg)
                    rate.sleep()



                #localization_msg = PoseStamped()

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

    rospy.init_node('localization', anonymous=True)
    rospy.loginfo("Successful initilization of node")


    quad = Localization()
    quad.start_position()
    quad.localize(algo= 'a-star', plot = True)
    #rospy.spin()
    #rospy.init_node('navgoal3')
