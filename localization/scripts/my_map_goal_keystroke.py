#!/usr/bin/env python   

import sys
import math
import rospy
from geometry_msgs.msg import PoseStamped
import keyboard  # using module keyboard

def get_keystroke():
    
    print("Waiting for input. Please press esc for exiting or space to a pass a new input") # if nothing is pressed


    while True:  # making a loop
        #try:  # used try so that if user pressed other than the given keys, then rectify
        if keyboard.is_pressed('space'):  # if key 'q' is pressed 
            print('Please enter a new goal pose')
            publish_goal()
            print("Successfully published above goal. Please press esc for exiting or space to a pass a new input") # if nothing is pressed

            
        elif keyboard.is_pressed('esc'):  # if enter is pressed, exit! 
            print('Exiting')
            break

        #else:
            #print("Waiting for input. Please press esc for exiting or space to a enter a new input") # if nothing is pressed

        #except:
            #print("Please press enter or space")  # if user pressed a key other than the given key the loop will break
            #return True


def publish_goal():
    
    goal = PoseStamped()
    pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 2)

    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"

    goal.pose.position.x = float(input("Enter x value: ")) 
    goal.pose.position.y = float(input("Enter y value: "))
    goal.pose.position.z = float(input("Enter z value: "))
    
    goal.pose.orientation.x = float(input("Enter x value of quaternion: "))
    goal.pose.orientation.y = float(input("Enter y value of quaternion: "))
    goal.pose.orientation.z = float(input("Enter z value of quaternion: "))
    goal.pose.orientation.w = float(input("Enter w value of quaternion: "))

    pub_goal.publish(goal)
    #rospy.loginfo('goal is', goal)

def main():
    # rospy.Rate(20)
    #while not rospy.is_shutdown():
    #     if rgoal:
    #         publish_goal(rgoal)
    #     # else:
    #     #     print('Final pose (x,y,z,yaw(deg)) = ')
    #     #     kgoal = input()
    #     #     pgoal = kgoal.split(',') 
    #     #     pgoal[0] = float(pgoal[0])
    #     #     pgoal[1] = float(pgoal[1])
    #     #     pgoal[2] = float(pgoal[2])
    #     #     pgoal[3] = float(pgoal[3])
    #     #     pub_goal.publish(pgoal)
    #     #     rospy.loginfo('pgoal is', pgoal)
    # rospy.sleep(1)
    
        rospy.init_node('my_map_goal')
        get_keystroke()
    #sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)

if __name__ == '__main__':
    main()
