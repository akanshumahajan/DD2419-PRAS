#! /usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from cv2 import aruco
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from geometry_msgs.msg import PoseStamped, Vector3
from crazyflie_gazebo.msg import Position
import matplotlib.pyplot as plt

from testObjectDetection import test_object
import tf2_geometry_msgs
import geometry_msgs

from image_to_camera import image_to_camera
from CameraInfoClass import CameraInfoClass
from PoseInfoClass import PoseInfoCLass


def transform_from_camera_to_map(data):
    goal_camera = PoseStamped()
    goal_camera.header.frame_id = "cf1/camera_link"

    goal_camera.pose.position.x = data[0][0][0]
    goal_camera.pose.position.y = data[0][0][1]
    goal_camera.pose.position.z = data[0][0][2]
    goal_camera.pose.orientation.x = 0
    goal_camera.pose.orientation.y = 0
    goal_camera.pose.orientation.z = 0
    goal_camera.pose.orientation.w = 1

    if not tf_buf.can_transform("cf1/base_link", 'cf1/camera_link', rospy.Time(0.0)):
        rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/base_link' % goal_camera.header.frame_id)
        return
    

    goal_base = tf_buf.transform(goal_camera,'cf1/base_link')
    # # goal_base = tf_buf.lookup_transform("cf1/base_link","cf1/camera_link",rospy.Time(0.0))
    
    # rospy.loginfo("goal_base")
    # rospy.loginfo(goal_base)

    if not tf_buf.can_transform("cf1/odom", 'cf1/base_link', rospy.Time(0.0)):
        rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % goal_base.header.frame_id)
        return
    
    goal_odom = tf_buf.transform(goal_base,'cf1/odom')
    
    if not tf_buf.can_transform("map", 'cf1/odom', rospy.Time(0.0)):
        rospy.logwarn_throttle(5.0, 'No transform from %s to map' % goal_odom.header.frame_id)
        return
    
    goal_map = tf_buf.transform(goal_odom,'map')
    
    
    print(goal_map)
    broadcaster1 = tf2_ros.StaticTransformBroadcaster()
    t1 = geometry_msgs.msg.TransformStamped()

    t1 = geometry_msgs.msg.TransformStamped()
    t1.header.stamp = rospy.Time(0.0)
    t1.header.frame_id = "map"
    t1.child_frame_id = "detectObject"

    t1.transform.translation = goal_map.pose.position
    t1.transform.rotation.x = 0.0#goal_map.pose.orientation
    t1.transform.rotation.y = 0.0
    t1.transform.rotation.z = 0.0
    t1.transform.rotation.w = 1.0

    broadcaster1.sendTransform(t1)
    

class image_converter:

  def __init__ (self):
    self.image_pub = rospy.Publisher("/myresult", Image, queue_size = 2)

    self.bridge = CvBridge ()
    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.callback)
    
    # Classes used to get the camera matrix and the pitch and roll off the drone in the odom frame
    self.camera_info = CameraInfoClass()
    self.pose_info = PoseInfoCLass()

    self.count = 0

  def callback (self, data):
    # Convert the image from OpenCV to ROS format
    try:
      cv_image = self.bridge.imgmsg_to_cv2 (data, "bgr8")
      # print(cv_image)
      res,label = test_object(cv_image)
      # print(res)
      color = (255, 0, 0) 
      
      if res.size!=0:
        cv2.rectangle(cv_image,(res[0][0],res[0][1]),(res[0][2],res[0][3]),color=color,thickness=2)
        
      # Find the objects' position
      obj_pos_and_angle = image_to_camera(res, label, self.camera_info.camera_matrix, pitch=0, roll=0)
      # TODO pitch/roll
      # print(self.camera_info.camera_matrix)
      roll, pitch, _ = self.pose_info.get_angles()
      obj_pos_and_angle = image_to_camera(res, label, self.camera_info.camera_matrix, pitch, roll)
      # print(obj_pos_and_angle)
      if obj_pos_and_angle:
        transform_from_camera_to_map(obj_pos_and_angle)
      
      print(roll,pitch,obj_pos_and_angle)
      
      # self.image_pub(cv_image)
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      
    except CvBridgeError as e:
      print(repr(e))


def detectX(args):
  # rospy.init_node ('detectX', anonymous = True)

  ic = image_converter()

  print( "Running ...")
  try:
    rospy.spin ()
  except KeyboardInterrupt:
    print ("Shutting down")

  cv2.destroyAllWindows ()



if __name__ == '__main__':
    # index = 31
    # cv_image = cv2.imread("saveImg/testDataset/img"+str(index)+".jpg")
    # box,label = test_object(cv_image)
    # print(box,label)
    # detectX(sys.argv)
    rospy.init_node ('detectX', anonymous = True)
    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)
    detectX(sys.argv)

