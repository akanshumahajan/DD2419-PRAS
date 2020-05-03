#!/usr/bin/env python
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import PoseWithCovariance, PoseStamped, TransformStamped

pose = None

marker_id = None

rospy.init_node('Driftmarker_pose_pub') 
listener = tf.TransformListener()
turtle_vel = rospy.Publisher('update_ready_odom_map', PoseStamped ,queue_size=1) 

tf_buf = tf2_ros.Buffer()
tf_lstn = tf2_ros.TransformListener(tf_buf)

""" def main():
    trans,rot = listener.lookupTransform(arucoFrame, "cf1/odom", rospy.Duration(0.1))
    rospy.loginfo("lookuped")
    cmd = PoseStamped()
    cmd.pose.position = trans
    cmd.pose.orientation = rot
    turtle_vel.publish(cmd) """

def moveDrone(arucoFrame):
    global currentPose, latestCmd
    marker = PoseStamped()
    marker.header.frame_id = arucoFrame

    roll, pitch, yaw = euler_from_quaternion((0, 0, 0, 1))

    roll -= math.pi/2
    yaw -= math.pi/2

    marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = quaternion_from_euler(
        roll, pitch, yaw)

    offset_odom = tf_buf.transform(marker, "cf1/odom", rospy.Duration(0.1))

    buildCmd(offset_odom)

def buildCmd(marker_odom):
    global latestCmd, currentPose
    cmd = PoseStamped()

    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = "cf1/odom"

    cmd.x = marker_odom.pose.position.x
    cmd.y = marker_odom.pose.position.y
    cmd.z = marker_odom.pose.position.z

    _, _, yaw = euler_from_quaternion((marker_odom.pose.orientation.x,
                                       marker_odom.pose.orientation.y,
                                       marker_odom.pose.orientation.z,
                                       marker_odom.pose.orientation.w))

    cmd.yaw = math.degrees(yaw)
    # rospy.loginfo("Updated cmd pose")
    latestCmd = cmd

if __name__ == "__main__":
    while not rospy.is_shutdown():
        for id in ["1", "2", "3", "4", "5", "6"]:
            arucoFrame = "aruco/detected" + id
            #t = tf.getLatestCommonTime(arucoFrame, "cf1/odom")
            if tf_buf.can_transform(arucoFrame, "cf1/odom", rospy.Duration(0.1)):
                rospy.loginfo("hudsahdusahd")
                turtle_vel.publish(cmd)
        rate = rospy.Rate(10)
        rate.sleep()

    