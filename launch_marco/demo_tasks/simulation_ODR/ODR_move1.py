#!/usr/bin/env python3


import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion



vel_angular = 0.5 
vel_x = 0.3
t0 = 0.0


rospy.init_node("mir_1_movements")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

command = Twist()
t0 = rospy.Time.now().to_sec()


rate = rospy.Rate(100)

while not rospy.is_shutdown():

    t1 = rospy.Time.now().to_sec()
    delta_t = t1-t0

    space = 1 #meter
    time_go_forward = 1/0.3
    time_rotate = math.pi/ 0.5
    sum_time= time_rotate + time_go_forward
    print ("time_go_forward", time_go_forward, "time_rotate", time_rotate, "sum_time", sum_time)
    
    if(delta_t) < time_go_forward:

        command.angular.z = 0.0
        command.linear.x = vel_x
        print ("GO FORWARD", "delta_t", delta_t, "vel_x", command.linear.x, "vel_angular", command.angular.z)

    else:
        
        if delta_t > sum_time:

        
            command.angular.z = 0.0
            command.linear.x = vel_x
            print ("GO BACK", "delta_t", delta_t, "vel_x", command.linear.x, "vel_angular", command.angular.z)
        
        else:
            command.angular.z = vel_angular
            command.linear.x = 0.0
            
            print ("ROTATE", "delta_t", delta_t, "vel_x", command.linear.x, "vel_angular", command.angular.z)

      

    # else:
    #     command.linear.x= 0.0
    #     command.angular.z = 0.0
    #     print ("I'M AT HOME", delta_t)

    pub.publish(command)
    rate.sleep()