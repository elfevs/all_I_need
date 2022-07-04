#!/usr/bin/env python


from click import command
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion



x1 = x2 =0.0
y1 = y2 = 0.0
roll_1 = pitch_1 = yaw_1 = roll_2 = pitch_2 = yaw_2 = 0.0
kP = 1.0 
kL = 1.0

    
def mir_2_callback(msg):

    global roll_2, pitch_2, yaw_2, x2, y2
    orientation_q = msg.pose.pose.orientation
    x2 = msg.pose.pose.position.x
    y2 = msg.pose.pose.position.y

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll_2, pitch_2, yaw_2) = euler_from_quaternion(orientation_list)

def  move_rot_callback(msg): 
    global roll_1, pitch_1, yaw_1, x1, y1
    orientation_q = msg.pose.pose.orientation
    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll_1, pitch_1, yaw_1) = euler_from_quaternion(orientation_list)

rospy.init_node("follow_mir_1")
pub_mir_2= rospy.Publisher("/mir2/mobile_base_controller/cmd_vel", Twist, queue_size=10)
sub_follow= rospy.Subscriber("/mir1/ground_truth", Odometry, move_rot_callback)
sub_mir_2= rospy.Subscriber("/mir2/ground_truth", Odometry, mir_2_callback)

speed = Twist()
rate = rospy.Rate(10)


while not rospy.is_shutdown():
    
    linear_distance = abs(x1-x2)
    angular_deviation = abs(yaw_1-yaw_2)
    Co= 0
    if linear_distance > 0.01:
        speed.linear.x = kL * linear_distance

    else:
        speed.linear.x = 0.0
    
    if angular_deviation > 0.1 :
        
        speed.angular.z = kP * angular_deviation

    else:
        
        speed.angular.z = 0.0

    print ("x1:", x1, "x2:", x2, "angular_deviation=", angular_deviation)


    pub_mir_2.publish(speed)
    rate.sleep()