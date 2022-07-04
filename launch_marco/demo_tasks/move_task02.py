#!/usr/bin/env python



from cmath import pi
from re import X
import rospy
import math
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

from tf.transformations import euler_from_quaternion

x = 0.0
y = 0.0
theta = 0.0

def move_turn_callback(msg):

    global x
    global y 
    global theta

    x= msg.pose.pose.position.x
    y= msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion( [rot_q.x, rot_q.y, rot_q.z, rot_q.w] )

rospy.init_node("speed_controller", anonymous= True)

sub= rospy.Subscriber ("/mobile_base_controller/odom", Odometry, move_turn_callback) #Stiamo salvando in continuola posizione del robot
pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)
speed= Twist()

rate = rospy.Rate(100)

goal = Point()

goal.x = 1.0
goal.y = 0.0
goal_theta = pi



while not rospy.is_shutdown():

    
    print(goal.x,x)
    if (goal.x < x):
        speed.linear.x = 0.0
        speed.angular.z = 0.2

        if abs(goal_theta-theta)> 0.1:
            speed.linear.x = 0.2
            speed.angular.z = 0.0
        else:
            continue

    else:
        speed.linear.x = 0.2
        speed.angular.z = 0.0
    
    
    pub.publish(speed)
    rate.sleep()
