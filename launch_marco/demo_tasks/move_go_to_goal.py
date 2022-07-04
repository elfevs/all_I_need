#!/usr/bin/env python



import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from math import atan2, degrees
from tf.transformations import euler_from_quaternion


# first try

x = 0.0
y = 0.0
theta = 0.0

def newOdom_callback (msg):
    global x
    global y
    global theta

    x= msg.pose.pose.position.x
    y= msg.pose.pose.position.y

    rot_q= msg.pose.pose.orientation
    (roll, pitch, theta)= euler_from_quaternion( [rot_q.x, rot_q.y, rot_q.z, rot_q.w])


rospy.init_node ("speed_controller")

sub= rospy.Subscriber ("/mobile_base_controller/odom", Odometry, newOdom_callback) #Stiamo salvando in continuola posizione del robot
pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)
speed= Twist()

rate = rospy.Rate(100)

goal = Point()
#TARGET 
goal.x= 1.0
goal.y= 0.0

while not rospy.is_shutdown():

    delta_x = abs(goal.x - x)
    delta_y = abs(goal.y - y)
    angle_to_goal = atan2 (delta_y, delta_x)
    displacement= degrees(abs(angle_to_goal-theta))

    print("x_goal:")
    print (goal.x)
    print ("y_goal:")
    print (goal.y)

    if displacement > 2.0:
        speed.linear.x = 0.0
        speed.angular.z = 0.1 #clockwise
        rospy.logdebug_once("Rotating...")
        print (displacement)
        print("x offset:")   
        print(delta_x) 
        print("y offset:") 
        print(delta_y) 
  

    else:
        if delta_x < 0.1 and delta_y < 0.1:


            print ("goal reached!")
            print("x offset:")   
            print(delta_x) 
            print("y offset:") 
            print(delta_y) 
            

            break



        else:
            speed.linear.x = 0.3 #go forward
            speed.angular.z = 0.0


            
            print ("go forward")
            print("x offset:")   
            print(delta_x) 
            print("y offset:") 
            print(delta_y) 

            


    pub.publish(speed)
    rate.sleep()
