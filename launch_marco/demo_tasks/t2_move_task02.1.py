#!/usr/bin/env python


from xmlrpc.client import boolean
import rospy
import math
import time
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


x = 0.0
y = 0.0
roll = pitch = yaw = 0.0
target_yaw = 180
target_rad= target_yaw * math.pi/180
kP = 0.5 
kL = 0.3
target_x = 1


def  move_rot_callback(msg): 
    global roll, pitch, yaw, x, y
    orientation_q = msg.pose.pose.orientation
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

rospy.init_node("Move_Rotate")
sub = rospy.Subscriber("/mobile_base_controller/odom", Odometry, move_rot_callback)
pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)

command = Twist()



rate = rospy.Rate(10)

while not rospy.is_shutdown():
    
    target_x = 1.0
    distance_x = (target_x-x)
    TF = 0

    if distance_x > 0.1:
        TF +=1
        command.angular.z = 0.0
        command.linear.x = kL * distance_x
        print ("yaw:", yaw, "x:", x, "target:", target_x)
    else:
        TF +=1
        
        command.angular.z = kP * (target_rad - yaw) #the speed is not constant
        command.linear.x = 0.0

    if (target_rad - yaw) < 0.01 and TF >= 1:
        command.angular.z = 0.0
        target_x_return = 0.0
        distance_x_return = (x- target_x_return)
        command.linear.x = distance_x_return * kL
        print ("x:", x, "target_x_ret:", target_x_return, "distance_x_return:", distance_x_return)

        if distance_x_return < 0.1: 
            break

    print (yaw, x)

    pub.publish(command)
    rate.sleep()