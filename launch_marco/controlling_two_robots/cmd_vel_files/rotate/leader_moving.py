#!/usr/bin/env python3

from cmath import cos, sin
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion



#mir 1 LEADER
#mir 2 FOLLOWER

x1 = 0.0
y1 = 0.0
roll_1 = pitch_1 = yaw_1 = 0.0

x2 = 0.0
y2 = 0.0
roll_2 = pitch_2 = yaw_2 = 0.0

lin_vel_x= 0.0
lin_vel_y = 0.0
ang_vel_z = 0.0
R= 2.0
x_goal= 2.0
FF= 0



rospy.init_node ("Leader_CFC")


def  mir1_pos_callback(msg): 
    global roll_1, pitch_1, yaw_1, x1, y1
    orientation_q = msg.pose.pose.orientation
    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z,orientation_q.w]
    (roll_1, pitch_1, yaw_1) = euler_from_quaternion(orientation_list)


sub_mir1_pos = rospy.Subscriber("/mir1/ground_truth", Odometry, mir1_pos_callback)


pub_mir1 = rospy.Publisher("/mir1/mobile_base_controller/cmd_vel", Twist, queue_size=10)


command_1 = Twist()

t0 = rospy.Time.now().to_sec()
rate = rospy.Rate(100)

while not rospy.is_shutdown():


    
    t1 = rospy.Time.now().to_sec()

    # current_angle=(t1-t0)* command_1.angular.z
    # current_angle_degree = math.degrees(current_angle)
    # Vx= command_1.linear.x* cos(current_angle)
    # Vy = command_1.linear.x* sin(current_angle)

    Vx= command_1.linear.x * cos(yaw_1)
    Vy = command_1.linear.x * sin(yaw_1)
    V= (Vx**2+Vy**2)**0.5

    yaw_1_degree = math.degrees(yaw_1)

    print ("Vx:", Vx, "Vy:", Vy, "V", V, "command_1.linear.x", command_1.linear.x)
    print ( "yaw:", yaw_1_degree)



    # go straight, rotate, go straight 

    if (x_goal-x1)> 0.1 and FF <= 1:
        
        print("I go forward")
        command_1.linear.x = 2.0
        command_1.linear.y = 0.0
        command_1.angular.z = 0.0

    else:
        # command_1.linear.x = 0.0
        # command_1.linear.y = 0.0
        # command_1.angular.z = 0.0
        FF = FF + 1
        if abs(yaw_1_degree) < 90.0:
            print ("I rotate till 70 degrees")
            command_1.linear.x = 2.0
            command_1.angular.z = command_1.linear.x/R
        else:
            print("I go forward")
            command_1.linear.x = 2.0
            command_1.linear.y = 0.0
            command_1.angular.z = 0.0




 

    pub_mir1.publish(command_1)
    rate.sleep()