#!/usr/bin/env python3


import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion




# mir 1 LEADER
# mir 2 FOLLOWER

pos_mir1_x = 0
pos_mir1_y = 0
roll_mir1 = pitch_mir1 = yaw_mir1 = 0.0

pos_mir2_x = 0
pos_mir2_y = 0
roll_mir2 = pitch_mir2 = yaw_mir2 = 0.0

lin_vel_mir1_x= 0.0
lin_vel_mir1_y = 0.0
ang_vel_mir1_z = 0.0
R = 2.0


rospy.init_node ("follower_centralize_formation_control")


#mir1_pos and mir_2_pos for checking the distance between the two robots and the orientation ------------------------------

def  mir1_pos_callback(msg): 
    global roll_mir1, pitch_mir1, yaw_mir1, pos_mir1_x, pos_mir1_y
    orientation_q = msg.pose.pose.orientation
    pos_mir1_x = msg.pose.pose.position.x
    pos_mir1_y= msg.pose.pose.position.y

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll_mir1, pitch_mir1, yaw_mir1) = euler_from_quaternion(orientation_list)


def  mir2_pos_callback(msg): 
    global roll_mir2, pitch_mir2, yaw_mir2, pos_mir2_x, pos_mir2_y
    orientation_q = msg.pose.pose.orientation
    pos_mir2_x = msg.pose.pose.position.x
    pos_mir2_y = msg.pose.pose.position.y

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll_mir2, pitch_mir2, yaw_mir2) = euler_from_quaternion(orientation_list)

#------------------------------------------------------------------------------------------------------
def  mir1_callback(msg): 
    global lin_vel_mir1_x , lin_vel_mir1_y, ang_vel_mir1_z
    lin_vel_mir1_x= msg.linear.x
    lin_vel_mir1_y = msg.linear.y
    ang_vel_mir1_z = msg.angular.z


sub_mir1_pos = rospy.Subscriber("/mir1/ground_truth", Odometry, mir1_pos_callback)
sub_mir2_pos = rospy.Subscriber("/mir2/ground_truth", Odometry, mir2_pos_callback)
sub_mir2_vel = rospy.Subscriber("/mir1/mobile_base_controller/cmd_vel", Twist, mir1_callback)

pub_mir1 = rospy.Publisher("/mir1/mobile_base_controller/cmd_vel", Twist, queue_size=10)
pub_mir2 = rospy.Publisher("/mir2/mobile_base_controller/cmd_vel", Twist, queue_size=10)

command_2 = Twist()
command_1 = Twist()

rate = rospy.Rate(100)

while not rospy.is_shutdown():

    omega = ang_vel_mir1_z
    
    r_1= (pos_mir1_x**2+pos_mir1_y**2)**0.5
    r_2= (pos_mir2_x**2+pos_mir2_y**2)**0.5

    D_x = (pos_mir2_x-pos_mir1_x)  
    D_y = (pos_mir2_y-pos_mir1_y)  
    distance = (D_x**2+ D_y**2)**0.5 
    
    
    command_1.linear.x = 1.0
    command_1.angular.z = 0.5
    command_2.linear.x = 1.5
    command_2.angular.z = 0.5

    # command_1.angular.z = 0.5
    
    # if omega >0:
    #     command_1.linear.x = R * omega
    #     command_2.linear.x= omega * (distance + R)
    #     command_2.angular.z = omega
    #     print ("clockwise rotation")
    # else:
    #     command_1.linear.x = omega * (distance + R)
    #     command_2.linear.x= omega * R
    #     command_2.angular.z = omega
    #     print ("unter-clockwise rotation")

    print("vel_1=", command_1.linear.x, "vel_2=", command_2.linear.x, "omega_1=", yaw_mir1,"omega_2=", yaw_mir2, "distance=", distance)
    print("-----------------------------------------------------------------------")

   
    pub_mir1.publish(command_1)
    pub_mir2.publish(command_2)
    rate.sleep()