#!/usr/bin/env python3


from turtle import position
import rospy 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

x=0
y=0
yaw=0

def odr_callback_move0(msg):
    global x, y, yaw
   

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orientation = msg.pose.pose.orientation
    

    orientation_list= [orientation.x,orientation.y, orientation.z, orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    

rospy.init_node("ODR_move0")

sub = rospy.Subscriber("/odom", Odometry, odr_callback_move0)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

command_vel = Twist()
t0 = rospy.Time.now().to_sec()
rate = rospy.Rate(10)

while not rospy.is_shutdown():

    target_x = 2.0
    t1= rospy.Time.now().to_sec()
    delta_t= t1-t0
    
    print ("time =",delta_t)
    
    print ("command_vel.linear.x", command_vel.linear.x)
    print ("x", x)
    
    if delta_t < 5:
        command_vel.linear.x = 0.3

    else:
        command_vel.linear.x = 0.0
        


    pub.publish(command_vel)
    rate.sleep()

 # I cannot recieve odom info and I cannot take the position info of the robot, like x ,y ...
 #insert another robot and controll them simoultaneously 