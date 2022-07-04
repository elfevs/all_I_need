#!/usr/bin/env python3


import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion




# mir 1 LEADER
# mir 2 FOLLOWER

x1 = 0
y1 = 0
roll_1 = pitch_1 = yaw_1 = 0.0

x2 = 1.0
y2 = 0.0
roll_2 = pitch_2 = yaw_2 = 0.0

lin_vel_x= 0.0
lin_vel_y = 0.0
ang_vel_z = 0.0



rospy.init_node ("follower_centralize_formation_control")


#mir1_pos and mir_2_pos for checking the distance between the two robots and the orientation ------------------------------

def  mir1_pos_callback(msg): 
    global roll_1, pitch_1, yaw_1, x1, y1
    orientation_q = msg.pose.pose.orientation
    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll_1, pitch_1, yaw_1) = euler_from_quaternion(orientation_list)


def  mir2_pos_callback(msg): 
    global roll_2, pitch_2, yaw_2, x2, y2
    orientation_q = msg.pose.pose.orientation
    x2 = msg.pose.pose.position.x
    y2 = msg.pose.pose.position.y

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll_2, pitch_2, yaw_2) = euler_from_quaternion(orientation_list)

#------------------------------------------------------------------------------------------------------
def  mir1_callback(msg): 
    global lin_vel_x , lin_vel_y, ang_vel_z
    lin_vel_x= msg.linear.x
    lin_vel_y = msg.linear.y
    ang_vel_z = msg.angular.z


sub_mir1_pos = rospy.Subscriber("/mir1/mobile_base_controller/odom", Odometry, mir1_pos_callback)
sub_mir2_pos = rospy.Subscriber("/mir2/mobile_base_controller/odom", Odometry, mir2_pos_callback)
sub_mir2_vel = rospy.Subscriber("/mir1/mobile_base_controller/cmd_vel", Twist, mir1_callback)


pub_mir2 = rospy.Publisher("/mir2/mobile_base_controller/cmd_vel", Twist, queue_size=10)

command_2 = Twist()


rate = rospy.Rate(100)

while not rospy.is_shutdown():
    
    r_1= (x1**2+y1**2)**0.5
    r_2= (x2**2+y2**2)**0.5

    D_x = (x2-x1)  
    D_y = (y2-y1)  
    distance = (D_x**2+ D_y**2)**0.5 *100000

    # knowing the orientation of the leader robot

    Vx= lin_vel_x*math.cos(yaw_1)
    Vy = lin_vel_x*math.sin(yaw_1)
    V_1 = (Vx**2+Vy**2)**0.5
    
    omega = ang_vel_z


    V2x = Vx+ omega * D_x
    V2y = Vy + omega * D_y
    V_2 = (V2x**2+V2y**2)**0.5

    if omega >0:
        command_2.linear.x= lin_vel_x * (R+distance *100000000000)
        command_2.angular.z = omega
        print ("clockwise rotation")
    else:
        command_2.linear.x= lin_vel_x * (R + distance *100000000000)
        command_2.angular.z = omega
        print ("unter-clockwise rotation")

    if command_2.angular.z != 0 :
        print ("Irotate")
    else:
        print ("I go forward")

    print ("distance:", distance)
    print ("V_1:", V_1,"V_2:",V_2,"distance:", distance,"theta", yaw_1, "vel_ang:", omega)
    print("-------------------------------------------------------------------------------------------")


    pub_mir2.publish(command_2)
    rate.sleep()