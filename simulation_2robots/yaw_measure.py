#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf import transformations


# mir 1 LEADER
# mir 2 FOLLOWER

rospy.init_node ("measure_yaw")


#mir1_pos and mir_2_pos for checking the distance between the two robots and the orientation ------------------------------

def  mir1_pos_callback(msg): 
    
    global roll_mir1, pitch_mir1, yaw_mir1, pos_mir1_x, pos_mir1_y,pos_mir1_z
    orientation_q = msg.pose.pose.orientation
    pos_mir1_x = msg.pose.pose.position.x
    pos_mir1_y= msg.pose.pose.position.y
    pos_mir1_z =msg.pose.pose.position.z

    orientation_list_1 = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll_mir1, pitch_mir1, yaw_mir1) = euler_from_quaternion(orientation_list_1)
        

def  mir2_pos_callback(msg): 
    global roll_mir2, pitch_mir2, yaw_mir2, pos_mir2_x, pos_mir2_y,pos_mir2_z, actual_position
    actual_position = msg.pose.pose
    orientation_q = msg.pose.pose.orientation
    pos_mir2_x = msg.pose.pose.position.x
    pos_mir2_y = msg.pose.pose.position.y
    pos_mir2_z =msg.pose.pose.position.z

    orientation_list_2 = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll_mir2, pitch_mir2, yaw_mir2) = euler_from_quaternion(orientation_list_2)


yaw_mir1 = 0.0
yaw_mir2 =0.0        
        
# sub_mir1_pos = rospy.Subscriber("/mir1/ground_truth", Odometry, mir1_pos_callback)
# sub_mir2_pos = rospy.Subscriber("/mir2/ground_truth", Odometry, mir2_pos_callback)

sub_mir1_pos = rospy.Subscriber("/mir1/mobile_base_controller/odom", Odometry, mir1_pos_callback)
sub_mir2_pos = rospy.Subscriber("/mir2/mobile_base_controller/odom", Odometry, mir2_pos_callback)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    print ("yaw1=", yaw_mir1, "yaw2=", yaw_mir2)
  
    
    
    rate.sleep()