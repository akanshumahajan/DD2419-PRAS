#!/usr/bin/env python
"""
Created on Wednesday Apr 22 17:14:00 2020

@author: Akanshu Mahajan
"""

#import json
import sys
import math
import rospy
import json
import string
import tf2_ros
import tf2_geometry_msgs
#import geometry_msgs.msg
import nav_msgs.msg

from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped
from crazyflie_driver.msg import Position
from crazyflie_driver.msg import Hover

from std_msgs.msg import Empty
from aruco_msgs.msg import MarkerArray
from aruco_msgs.msg import Marker

#from binary_map_4n import localization_algo


class Localization:

    def __init__(self):
        
        ## Pull necessary ROS parameters from launch file:
        # Read goal message topic
        #param1 = rospy.search_param("marker_map_message_topic") 
        #self.marker_map_message_topic = rospy.get_param(param1)       # /aruco_map_pose

        # Read Aruco message topic
        # param2 = rospy.search_param("marker_odom_message_topic")
        # self.marker_odom_message_topic = rospy.get_param(param2) #/aruco_odom_pose_tf

        # self.marker_odom_message_topic = '/aruco_odom_pose_tf'
        # Initialize callback variables
        #self.marker_map_msg = None
        # self.marker_odom_msg = None

        # Initialize class variables
        #self.localization_msg = None

        # Establish subscription to control message
        #rospy.Subscriber(self.marker_map_message_topic, PoseStamped, self.marker_map_msg_callback)
        # rospy.Subscriber(self.marker_odom_message_topic, PoseStamped, self.marker_odom_msg_callback)
        
        # Delay briefly for subscriber to find message
        # rospy.sleep(2)

        # # Establish publisher of converted Twist message
        # self.pub = rospy.Publisher(self.localization_msg_topic, PoseStamped, queue_size=10)

        # Initilize tf2 broadcaster and transform message
        self.br = tf2_ros.TransformBroadcaster()

        # self.br = tf2_ros.StaticTransformBroadcaster()
        # Initialize listener for estimated pose of vehicle in map frame
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # rospy.sleep(2)

    # def marker_map_msg_callback(self, marker_map_msg):
    #     self.marker_map_msg = marker_map_msg

    # def marker_odom_msg_callback(self, marker_odom_msg):
    #     self.marker_odom_msg = marker_odom_msg
        # self.bool_static = False

    def marker_map(self, m):

        self.x_map, self.y_map, self.z_map = m['pose']['position']
        # temp = self.x_map
        # self.x_map = self.y_map
        # self.y_map = -temp
        self.roll_map, self.pitch_map, self.yaw_map = m['pose']['orientation']       
        

    def diff(self):

        print("Translation map is")
        print(self.x_map)
        print(self.y_map)
        print(self.z_map)

        # temp = self.x_odom
        # self.x_odom = self.y_odom
        # self.y_odom = -temp

        self.x_diff = self.x_map - self.x_odom
        self.y_diff = self.y_map - self.y_odom
        self.z_diff = self.z_map - self.z_odom

        print("Translation difference odom is")
        print(self.x_diff)
        print(self.y_diff)
        print(self.z_diff)


        print("Roll_map:")
        print(self.roll_map)
        print(self.pitch_map)
        print(self.yaw_map)

        print("Roll_odom:")
        print(self.roll_odom)
        print(self.pitch_odom)
        print(self.yaw_odom)

        temp_angle = self.roll_odom
        self.roll_odom = self.pitch_odom
        self.pitch_odom = self.yaw_odom
        # temp_angle = self.pitch_odom
        # self.pitch_odom = self.yaw_odom
        self.yaw_odom = temp_angle
        # self.pitch_odom = self.pitch_odom
        # self.pitch_odom = temp_angle

        print("Roll_odom updated:")
        print(self.roll_odom)
        print(self.pitch_odom)
        print(self.yaw_odom)



        self.roll_diff = self.roll_map - self.roll_odom
        self.pitch_diff = self.pitch_map - self.pitch_odom
        self.yaw_diff = self.yaw_map - self.yaw_odom


        print("Roll_diff:")
        print(self.roll_diff)
        print(self.pitch_diff)
        print(self.yaw_diff)


        if self.yaw_diff <= -90 and self.yaw_diff > -180:
            temp = self.x_diff
            self.x_diff = self.y_diff
            self.y_diff = -temp

        elif self.yaw_diff <= -180 and self.yaw_diff > -270:

            self.x_diff = - self.x_diff
            self.y_diff = -self.y_diff

        elif self.yaw_diff <= -270 and self.yaw_diff > -360:

            temp = self.x_diff
            self.x_diff = -self.y_diff
            self.y_diff = temp


        # rospy.loginfo(self.x_map)
        # rospy.loginfo(self.x_odom)
        # self.x_diff = self.x_map - self.x_odom
        # self.y_diff = self.y_map - self.y_odom
        # self.z_diff = self.y_map - self.y_odom

        # self.roll_diff = 0
        # self.pitch_diff = 0
        # self.yaw_diff = 0


    def odom_to_map(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'map'
        t.child_frame_id = 'cf1/odom'
        t.transform.translation.x = self.x_diff
        t.transform.translation.y = self.y_diff
        t.transform.translation.z = self.z_diff
        
        (t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w) = quaternion_from_euler(math.radians(self.roll_diff),
                                                     math.radians(self.pitch_diff),
                                                     math.radians(self.yaw_diff),'rzxy')

        temp = t.transform.rotation.y
        t.transform.rotation.y = t.transform.rotation.z
        t.transform.rotation.z = temp

        # (t.transform.rotation.x,
        # t.transform.rotation.y,
        # t.transform.rotation.z,
        # t.transform.rotation.w) = (0,0,-0.7071068,0.7071068)
        
        # (t.transform.rotation.x,
        # t.transform.rotation.y,
        # t.transform.rotation.z,
        # t.transform.rotation.w) = quaternion_from_euler(math.radians(0),
        #                                              math.radians(0),
        #                                              math.radians(-90),'ryxz')


        print(t)
        # self.br2.sendTransform(t)
        # rospy.sleep(3600)

        return t

    def odom_to_map_static(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'map'
        t.child_frame_id = 'cf1/odom'
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0
        
        (t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w) = (0,0,0,1)
        print(t)

        # self.br.sendTransform(t)
        # rospy.sleep(1)
        return t



    def marker_odom(self, marker):
        
        trans = None
        transform = TransformStamped()
        trans_static = False
        trans_detected = False

        source_frame = "aruco/detected" + str(marker['id'])

        try:
            trans = self.tfBuffer.lookup_transform('cf1/odom', source_frame,rospy.Time(0), rospy.Duration(1.0))
            rospy.loginfo('Success of lookup transfrom from %s to cf1/odom' % source_frame)

        except:
            #trans = self.tfBuffer.lookup_transform(self.map_frame, self.est_veh_pose_frame, rospy.Time(0), rospy.Duration(1.0))
            rospy.loginfo('Failure of lookup transfrom from %s to cf1/odom' % source_frame)
        
        if trans!= None:

            #print (m)
            self.x_odom = trans.transform.translation.x
            self.y_odom = trans.transform.translation.y
            self.z_odom = trans.transform.translation.z
            
            print("Translation odom is")
            print(self.x_odom)
            print(self.y_odom)
            print(self.z_odom)

            self.x_quat_odom = round(trans.transform.rotation.x, 1)
            self.y_quat_odom = round(trans.transform.rotation.y, 1)
            self.z_quat_odom = round(trans.transform.rotation.z, 1)
            self.w_quat_odom = round(trans.transform.rotation.w, 1)      

            print(self.x_quat_odom)
            print(self.y_quat_odom)
            print(self.z_quat_odom)
            print(self.w_quat_odom)

            (self.roll_odom,
            self.pitch_odom,
            self.yaw_odom,) = euler_from_quaternion([self.x_quat_odom, self.y_quat_odom, self.z_quat_odom, self.w_quat_odom],axes='rzxy') #output in radians, Check the axis!!!! rzyx
            
            # print(self.roll_odom)
            # print(self.pitch_odom)
            # print(self.yaw_odom)
            
            self.roll_odom= round(self.roll_odom,3)
            self.pitch_odom= round(self.pitch_odom,3)
            self.yaw_odom= round(self.yaw_odom,3)

            # Converting into degrees to subtract from map orientation angles which are in euler i.e in degrees
            self.roll_odom = math.degrees(self.roll_odom)
            self.pitch_odom = math.degrees(self.pitch_odom)
            self.yaw_odom = math.degrees(self.yaw_odom)

            # print(self.roll_odom)
            # print(self.pitch_odom)
            # print(self.yaw_odom)

            self.marker_map(marker)
            self.diff()
            transform = self.odom_to_map()
            trans_detected = True
            rospy.loginfo("Done")
            # self.br.sendTransform(transform)
            # rospy.sleep(1)

        else:
            
            # if self.bool_static is False:
            rospy.loginfo("No transform available")
            # transform = self.odom_to_map_static()
            # trans_static = True
            # rospy.loginfo("Published static transform")
            # print(transform)
            # self.br.sendTransform(transform)
            # rospy.sleep(1)
            #     self.bool_static =True

            #rospy.sleep(3)

        return trans_static, trans_detected , transform


    def main(self, argv=sys.argv, transform_bool = False):
        
        # rate = rospy.Rate(10)  # Hz
        # transform = self.odom_to_map_static()
        # rospy.loginfo("Done Static transform")
        # print(transform)
        # self.br.sendTransform(transform)
        # rospy.sleep(1)
        transform = TransformStamped()
        trans_static = False
        trans_detected = False
        while transform_bool is False:

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

            #transform = self.marker_odom()
        
            for m in self.world['markers']:
            #    transforms = [transform_from_marker(m)]
                trans_static, trans_detected , transform = self.marker_odom(m)
                
                if trans_detected is True:
                    rospy.loginfo("Transform available")

                    self.br.sendTransform(transform)
                    rospy.sleep(1)

                    # transform_bool = True       # Set to True to get the first transform and keep on publishing that
                    transform_bool = False

                    break
                elif trans_static is True:
                    rospy.loginfo("No transform available")
                    # self.br.sendTransform(transform)
                    # self.odom_to_map_static()
                    rospy.sleep(1)
                    # rospy.loginfo("Published static transform")

                    transform_bool = False
        
        return transform
        # rospy.spin()


        # rate.sleep()

if __name__ == '__main__':

    transform = TransformStamped()
    rospy.init_node('map_to_odom', anonymous=True)
    rospy.loginfo("Successful initilization of node map_to_odom")

    odom_to_map = Localization()

    transform = odom_to_map.main()
    
    rate = rospy.Rate(10)
    br2 = tf2_ros.StaticTransformBroadcaster()
    # br2.sendTransform(transform)
    # rospy.sleep(3)
    # rospy.spin()
    while not rospy.is_shutdown():
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'cf1/odom'

        br2.sendTransform(transform)
        rospy.sleep(1)
        print(transform)
        rate.sleep
