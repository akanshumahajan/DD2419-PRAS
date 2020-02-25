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

from crazyflie_gazebo.msg import Position
import matplotlib.pyplot as plt

from testObjectDetection import test_object

class image_converter:

  def __init__ (self):
    self.image_pub = rospy.Publisher("/myresult", Image, queue_size = 2)

    self.bridge = CvBridge ()
    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.callback)

    self.count = 0

  def callback (self, data):
    # Convert the image from OpenCV to ROS format
    try:
      cv_image = self.bridge.imgmsg_to_cv2 (data, "bgr8")
      # print(cv_image)
      res,label = test_object(cv_image)
      # print(res)
      color = (255, 0, 0) 
      # cv2.imwrite("cv_image.jpg",cv_image)
      if res.size!=0:
        cv2.rectangle(cv_image,(res[0][0],res[0][1]),(res[0][2],res[0][3]),color=color,thickness=2)
      
      # self.image_pub(cv_image)
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      
    except CvBridgeError as e:
      print (e)


def detectX(args):
  rospy.init_node ('detectX', anonymous = True)

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
    detectX(sys.argv)
