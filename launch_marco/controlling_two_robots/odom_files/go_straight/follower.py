#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


x1 = 0.0
y1 = 0.0
roll_1 = pitch_1 = yaw_1 = 0.0

x2 = 0.0
y2 = 0.0
roll_2 = pitch_2 = yaw_2 = 0.0

t1=0


#mir 1 LEADER
#mir2 FOLLOWER

rospy.init_node("follower_two_mir_together")

def  mir1_callback(msg): 
    global roll_1, pitch_1, yaw_1, x1, y1
    orientation_q = msg.pose.pose.orientation
    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll_1, pitch_1, yaw_1) = euler_from_quaternion(orientation_list)



def  mir2_callback(msg): 
    global roll_2, pitch_2, yaw_2, x2, y2
    orientation_q = msg.pose.pose.orientation
    x2 = msg.pose.pose.position.x
    y2 = msg.pose.pose.position.y

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll_2, pitch_2, yaw_2) = euler_from_quaternion(orientation_list)


sub_mir1 = rospy.Subscriber("/mir1/mobile_base_controller/odom", Odometry, mir1_callback)
sub_mir2 = rospy.Subscriber("/mir2/mobile_base_controller/odom", Odometry, mir2_callback)

pub_mir2 = rospy.Publisher("/mir2/mobile_base_controller/cmd_vel", Twist, queue_size=10)



command = Twist()


rate = rospy.Rate(100)

while not rospy.is_shutdown():

    if (x2-x1)>0.01:
        command.linear.x = 0.8
        print ("DeltaX=", (x2-x1))
        print("moving")
    else:
        print ("waiting")
        command.linear.x=0.0
    

    pub_mir2.publish(command)
    rate.sleep()