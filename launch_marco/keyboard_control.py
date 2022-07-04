#!/usr/bin/env python




import rospy
import math
import time
from geometry_msgs.msg import Twist, Pose, Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

roll= pitch = yaw=  x= y

def controller_keyboard_callback (msg):
    global roll, pitch, yaw, x, y
    orientation_q = msg.pose.pose.orientation
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

rospy.init_node("keyboard_controller")

sub = rospy.Subscriber("/ground_truth", Odometry, controller_keyboard_callback)
pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)