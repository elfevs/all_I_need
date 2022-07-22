#!/usr/bin/env python3


import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


x1 = 0.0
y1 = 0.0
roll_1 = pitch_1 = yaw_1 = 0.0
target_yaw = -180
target_rad=  math.radians(target_yaw)
kP = -0.3 
kL = 0.3
target_x = 1

v_max = 0.3

state = 0
state_1 = 0

def  move_rot_callback(msg): 
    global roll_1, pitch_1, yaw_1, x1, y1
    orientation_q = msg.pose.pose.orientation
    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll_1, pitch_1, yaw_1) = euler_from_quaternion(orientation_list)

rospy.init_node("mir_1_movements")
sub = rospy.Subscriber("/mir1/ground_truth", Odometry, move_rot_callback)
pub = rospy.Publisher("/mir1/mobile_base_controller/cmd_vel", Twist, queue_size=10)

command = Twist()



rate = rospy.Rate(50)

while not rospy.is_shutdown():
    

    target_x = 1.0
    distance_x = (target_x-x1)
    target_x_return = 0.0
    distance_x_return = (x1- target_x_return )
    angular_deviation = abs(target_rad - yaw_1)
    
    
   
            
    if angular_deviation > 0.1:
   
        command.angular.z = kP * angular_deviation #the speed is not constant
        command.linear.x = 0.0
    else :
       break

    print ("I ROTATE",
        "angular deviation:", angular_deviation, 
        "distance_x", distance_x, "distance_x_return",
        distance_x_return, "speed_x=", command.linear.x,"state", state, "state_1", state_1)

    pub.publish(command)
    rate.sleep()