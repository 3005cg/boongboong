#!/usr/bin/env python
import rospy
import numpy
import sys, select, os
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import cos, sin, tan,pi
import tf
V,W = 0.0,0.0
x,y,heading = 0.0,0.0,0.0
previousTime = 0
def control_input(msg):
    global V,W
    print('callback!')
    V = msg.linear.x * 0.0001
    W = -msg.angular.z *(pi/180) * 0.01 


def odom_pub():
    global V,W,x,y,heading
    rospy.init_node('odom_pub_node', anonymous=True)
    print('odom_pub_node!')
    sub = rospy.Subscriber('cmd_vel', Twist, control_input)
    rate = rospy.Rate(10) 
    odom_publisher = rospy.Publisher('odom_msg', Odometry,queue_size=1)
    previousTime = rospy.Time.now()
    while not rospy.is_shutdown():

        delta_time = (rospy.Time.now() - previousTime).to_sec()

        delta_transltation = V*delta_time
        delta_angle = W*delta_time

        heading = heading + delta_angle

        x = x + delta_transltation*cos(heading)
        y = y + delta_transltation*sin(heading)


        pubMsg = Odometry()
        pubMsg.header.stamp = rospy.Time.now()
        pubMsg.header.frame_id = '/world'
        pubMsg.pose.pose.position.x = x
        pubMsg.pose.pose.position.y = y
        quat = tf.transformations.quaternion_from_euler(0.0,0.0,heading)
        pubMsg.pose.pose.orientation.x = quat[0]
        pubMsg.pose.pose.orientation.y = quat[1]
        pubMsg.pose.pose.orientation.z = quat[2]
        pubMsg.pose.pose.orientation.w = quat[3]
        odom_publisher.publish(pubMsg)
        rate.sleep()

if __name__ == '__main__':
    try:
        odom_pub()
    except rospy.ROSInterruptException:
        pass