#!/usr/bin/env python3

import rospy
import math
import time
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


x = y = z = 0.0
roll = pitch = yaw = 0.0
target_yaw = 180
kP = 0.5 
target_x = 1
def  move_rot_callback(msg): 
    global roll, pitch, yaw, x, y, z
    orientation_q = msg.pose.pose.orientation

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

rospy.init_node("Move_Rotate")
sub = rospy.Subscriber("/mobile_base_controller/odom", Odometry, move_rot_callback)
pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)

command = Twist()



rate = rospy.Rate(10)

while not rospy.is_shutdown():
    distance_x = abs(target_x-x)
    if distance_x > 0.15:
        command.linear.x = 0.3
        print (yaw, distance_x)
    else:
        target_rad= target_yaw * math.pi/180
        command.angular.z = kP * (target_rad - yaw) #the speed is not constant

        if (target_rad - yaw) < 0.1:
            command.angular.z = 0.0
            target_x_return = 0.0
            distance_x_return = (x- target_x_return)
            
            if distance_x_return > 0.1:
                command.linear.x = 0.3
            else:
                break

        print (yaw, x)

        pub.publish(command)
        rate.sleep()