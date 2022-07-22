#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf import transformations
import numpy

# mir 1 LEADER
# mir 2 FOLLOWER

rospy.init_node ("follower_centralize_formation_control")


#mir1_pos and mir_2_pos for checking the distance between the two robots and the orientation ------------------------------

def  mir1_pos_callback(msg): 
    global roll_mir1, pitch_mir1, yaw_mir1, pos_mir1_x, pos_mir1_y
    orientation_q = msg.pose.pose.orientation
    pos_mir1_x = msg.pose.pose.position.x
    pos_mir1_y= msg.pose.pose.position.y

    orientation_list_1 = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll_mir1, pitch_mir1, yaw_mir1) = euler_from_quaternion(orientation_list_1)
    


def  mir2_pos_callback(msg): 
    global roll_mir2, pitch_mir2, yaw_mir2, pos_mir2_x, pos_mir2_y
    orientation_q = msg.pose.pose.orientation
    pos_mir2_x = msg.pose.pose.position.x
    pos_mir2_y = msg.pose.pose.position.y

    orientation_list_2 = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll_mir2, pitch_mir2, yaw_mir2) = euler_from_quaternion(orientation_list_2)

#------------------------------------------------------------------------------------------------------
def  mir1_callback(msg): 
    global lin_vel_mir1_x , lin_vel_mir1_y, ang_vel_mir1_z
    lin_vel_mir1_x= msg.linear.x
    lin_vel_mir1_y = msg.linear.y
    ang_vel_mir1_z = msg.angular.z


pos_mir1_x = 0
pos_mir1_y = 0
roll_mir1 = pitch_mir1 = yaw_mir1 = 0.0

pos_mir2_x = 0
pos_mir2_y = 0
roll_mir2 = pitch_mir2 = yaw_mir2 = 0.0

lin_vel_mir1_x= 0.0
lin_vel_mir1_y = 0.0
ang_vel_mir1_z = 0.0

t0 =0
t1 =0
desired_distance = 1.0 



sub_mir1_pos = rospy.Subscriber("/mir1/ground_truth", Odometry, mir1_pos_callback)
sub_mir2_pos = rospy.Subscriber("/mir2/ground_truth", Odometry, mir2_pos_callback)
sub_mir2_vel = rospy.Subscriber("/mir1/mobile_base_controller/cmd_vel", Twist, mir1_callback)


pub_mir2 = rospy.Publisher("/mir2/mobile_base_controller/cmd_vel", Twist, queue_size=10)

command_2 = Twist()


rate = rospy.Rate(100)

while not rospy.is_shutdown():
    
    
    
    
    omega = ang_vel_mir1_z 
    
    position_Gframe_mir1 = numpy.array([pos_mir1_x, pos_mir1_y, 0])
    position_Gframe_mir2 = numpy.array([pos_mir2_x, pos_mir2_y, 0])
    
    distance = (position_Gframe_mir2 - position_Gframe_mir1)
    
    
    vel_mir1_Lframe = numpy.array([lin_vel_mir1_x, lin_vel_mir1_y, omega])
    
    Rotation_matrix_mir1 = numpy.array([[math.cos(yaw_mir1), math.sin(yaw_mir1), 0],
                                        [-math.sin(yaw_mir1), math.cos(yaw_mir1), 0],
                                        [0, 0, 1]])
    
    inv_Rotation_matrix_mir1 = numpy.linalg.inv(Rotation_matrix_mir1)
    
    
    vel_mir1_Gframe = numpy.dot(inv_Rotation_matrix_mir1,vel_mir1_Lframe)
    
    transformations.rotation_matrix
    
    vel_mir2_Gframe = vel_mir1_Gframe + numpy.dot(omega,distance)
    
    Rotation_matrix_mir2 = numpy.array([[math.cos(yaw_mir1), math.sin(yaw_mir1), 0],
                                        [-math.sin(yaw_mir1), math.cos(yaw_mir1), 0],
                                        [0, 0, 1]])
    # I put the yaw of the mir1 because the yaw change sign in a certain angle
        
    vel_mir2_Lframe = numpy.dot(Rotation_matrix_mir2,vel_mir2_Gframe)
    
    
    command_2.linear.x = vel_mir2_Lframe[0]
    command_2.linear.y = vel_mir2_Lframe[1]
    command_2.angular.z = vel_mir2_Lframe[2]
    
    #----------------------------------------------
    # In the following codes, the result is like follower_following.py file
    # command_2.linear.x = (vel_mir2_Gframe[0]**2+vel_mir2_Gframe[1]**2)**0.5
    # command_2.angular.z = vel_mir2_Gframe[2]
    #-------------------------------------------------
    
    
    
    command_mir2 = numpy.array([command_2.linear.x, command_2.linear.y, command_2.angular.z]) 
    
    
    print ("vel_1", vel_mir1_Lframe, "vel_2", command_mir2 )
   
    print ("distance", distance)        


    pub_mir2.publish(command_2)
    rate.sleep()