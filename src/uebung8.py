#!/usr/bin/env python

import rospy
import math
from autominy_msgs.msg import SteeringCommand, SpeedCommand
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import rosbag
import time

rospy.init_node("uebung8", anonymous=True)

x1=0.0
y1=0.0
currSteering=0.0
currSpeed=0.0
odom_quaternion=[0.0, 0.0, 0.0, 0.0]

def getGPS(raw_msg):
    global x1, y1
    x1=raw_msg.pose.pose.position.x
    y1=raw_msg.pose.pose.position.y
    return x1, y1

def findSteering(raw_msg):
    global currSteering
    currSteering=raw_msg
    return currSteering

def findSpeed(raw_msg):
    global currSpeed
    currSpeed=raw_msg
    return currSpeed

rate = rospy.Rate(100) #100hz
publish_odometry = rospy.Publisher("nav_msgs/Odometry", Odometry, queue_size=100)
subscribe_GPS = rospy.Subscriber("/communication/131/localization", Odometry, getGPS, queue_size=10)
subscribe_steering = rospy.Subscriber("/sensors/steering", SteeringCommand, findSteering, queue_size=100)
subscribe_speed = rospy.Subscriber("/sensors/speed", SpeedCommand, findSpeed, queue_size=100)

current_time = rospy.get_time()
previous_time = rospy.get_time()

while not rospy.is_shutdown():

    rospy.sleep(5)

    #set header and child_frame_id
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = '/map'
    odom.child_frame_id = '/base_link'

    #position
    odom.pose.pose.position.x = x1
    odom.pose.pose.position.y = y1
    odom.pose.pose.orientation = odom_quaternion

    #calculations from assignment
    current_time = rospy.get_time()
    delta_t = current_time-previous_time
    xVelocity = currSpeed*(math.cos(currSteering))
    yVelocity = currSpeed*(math.sin(currSteering))
    angleVelocity = (currSpeed/0.27)*math.tan(currSteering)

    old_x = x1
    old_y = y1

    old_angleV = angleVelocity
    new_x = old_x + (delta_t*xVelocity)
    new_y = old_y + (delta_t*yVelocity)
    new_theta= old_angleV + (delta_t*angleVelocity)

    #velocity
    odom.child_frame_id = '/base_link'
    odom.twist.twist.linear.x = xVelocity
    odom.twist.twist.linear.y = yVelocity
    odom.twist.twist.angular.z = angleVelocity

    #publish odometry, current time becomes previous time
    publish_odometry.publish(odom) 
    previous_time = current_time