#!/usr/bin/env python3

from unicodedata import unidata_version
import rospy
import math
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf import transformations
import numpy
from cartesian_controller import cartesian_controller

# mir 1 LEADER
# mir 2 FOLLOWER

rospy.init_node ("follower_centralize_formation_control")


#mir1_pos and mir_2_pos for checking the distance between the two robots and the orientation ------------------------------

def  mir1_pos_callback(msg): 
    actual_pose = msg.pose.pose
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


sub_mir1_pos = rospy.Subscriber("/mir1/ground_truth", Odometry, mir1_pos_callback)
sub_mir2_pos = rospy.Subscriber("/mir2/ground_truth", Odometry, mir2_pos_callback)
sub_mir2_vel = rospy.Subscriber("/mir1/mobile_base_controller/cmd_vel", Twist, mir1_callback)


pub_mir2 = rospy.Publisher("/mir2/mobile_base_controller/cmd_vel", Twist, queue_size=10)

command_2 = Twist()




rate = rospy.Rate(100)

while not rospy.is_shutdown():
    
    
    position_Gframe_mir1 = numpy.array([pos_mir1_x, pos_mir1_y, 0])
    position_Gframe_mir2 = numpy.array([pos_mir2_x, pos_mir2_y, 0])
    
    Kv = 0.5
    Ky = 0.45
    Kx = 0.3
    act_pose = Pose()
    act_pose.orientation.x =0
    act_pose.orientation.y =0 
    act_pose.orientation.z =0
    act_pose.orientation.w =0
        
    set_pose_x= position_Gframe_mir1[0]
    set_pose_y= position_Gframe_mir1[1]
    w_target = ang_vel_mir1_z
    v_target= lin_vel_mir1_x
    phi_target= yaw_mir1
    phi_act = transformations.euler_from_quaternion([act_pose.orientation.x, act_pose.orientation.y, act_pose.orientation.z, act_pose.orientation.w])


    e_x = (set_pose_x-act_pose.position.x)
    e_y = (set_pose_y - act_pose.position.y)
    rospy.loginfo_throttle(1,[id,e_x,e_y])
    e_local_x = math.cos(phi_act[2]) * e_x + math.sin(phi_act[2]) * e_y
    e_local_y = math.cos(phi_act[2]) * e_y - math.sin(phi_act[2]) * e_x


    u_w = w_target + v_target * Kv * e_local_y + Ky * math.sin(phi_target-phi_act[2])
    u_v = v_target * math.cos(phi_target-phi_act[2]) + Kx*e_local_x
        
        
    command_2.linear.x = u_v
    
    command_2.angular.z =u_w

    
    cartesian_controller()

        
    #print ( "vel_1", command_mir1 )
    #print ( "vel_2", command_mir2 )
    #print ( "yaw_2", pos_ )
   
 
    print ("vel_2_x:", command_2.linear.x, "vel_2_ang:", command_2.angular.z)
    print (yaw_mir2, phi_target)
    print ("--------------------------------------------------")
    


    pub_mir2.publish(command_2)
    rate.sleep()