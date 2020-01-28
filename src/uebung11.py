#!/usr/bin/env python

import rospy
import math
import time
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from autominy_msgs.msg import NormalizedSteeringCommand, SteeringCommand, SpeedCommand
from map import Lane, Map, MapVisualization
from steering_pid import SteeringPID

rospy.init_node("uebung11")

currx=0.0
curry=0.0
currAngle=0.0
speed_msg = SpeedCommand()
steering_msg = SteeringCommand()

#from map.py
lane_1 = np.load("lane1.npy")
lane_2 = np.load("lane2.npy")
lanes = [
    Lane(lane_1[[0, 50, 209, 259, 309, 350, 409, 509, 639, 750, 848, 948, 1028, 1148, 1200, 1276], :]),
    Lane(lane_2[[0, 50, 100, 150, 209, 400, 600, 738, 800, 850, 900, 949, 1150, 1300, 1476], :])]

def getOdometry(msg): 

    global currx, curry, currAngle, speed_msg, steering_msg
    odo=msg

    #from map.py: find lookahead point
    point = np.array([odo.pose.pose.position.x, odo.pose.pose.position.y])
    lookahead, param = lanes[1].lookahead_point(point, 1.5)
    #lookahead=MapVisualization.getOdometry(odo)

    #current points
    currx=odo.pose.pose.position.x
    curry=odo.pose.pose.position.y
    #lookahead points
    lookaheadx=lookahead[0][0]
    lookaheady=lookahead[0][1]

    #delta y and delta x
    dy=lookaheady-curry
    dx=lookaheadx-currx

    #calculate delta
    delta=math.atan2(dy, dx)

    #calculate theta
    theta=math.asin(odo.pose.pose.orientation.z)*2

    #angle differenct to be put in PID
    angleDifference=delta-theta

    #set speed
    speed_msg.value = 0.3
    speed_pub.publish(speed_msg)

    steering_msg.value=angleDifference
    steering_pub.publish(steering_msg)

steering_pub = rospy.Publisher("/actuators/steering", SteeringCommand, queue_size=10)
speed_pub = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=100)
localization_sub = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, getOdometry, queue_size=1)

rospy.spin()