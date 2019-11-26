#!/usr/bin/env python

import rospy
import math
from autominy_msgs.msg import NormalizedSteeringCommand
from autominy_msgs.msg import SpeedCommand
from autominy_msgs.msg import Tick
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose

rospy.init_node("uebung6", anonymous=True)

x1=0.0
y1=0.0
x2=0.0
y2=0.0
totalTicks=0.0
position1 = [0.0, 0.0]
position2 = [0.0, 0.0]
endTicks = 0.0
slope = 0.0

def getGPS(raw_msg):
    global x1, y1
    x1=raw_msg.pose.pose.position.x
    y1=raw_msg.pose.pose.position.y
    #rospy.loginfo("Position x: " + str(x1)+" , Position y: " + str(y1))
    return x1, y1

def countTicks(ticks):
    global totalTicks, endTicks
    totalTicks=ticks.value
    endTicks += totalTicks
    #rospy.loginfo("Total ticks: " + str(endTicks))

def euclidean_distance(x1, y1, x2, y2):
    #global x1, y1, x2, y2
    distance = math.sqrt(((x1 - y1) ** 2) + ((x2 - y2) ** 2))
    print("euclidean distance: " + str(distance))
    return distance

def calculate_slope(x1, y1, x2, y2):
    slope = (y2-y1)/(x2-x1)
    print ("slope: " + str(slope))
    return slope

def resetVariables():
    global x1, y1, x2, y2, totalTicks, position1, position2, endTicks, slope, euclideanDistance, current_slope, current_angle, radius, distanceDriven, ratio
    x1=0.0
    y1=0.0
    x2=0.0
    y2=0.0
    totalTicks=0.0
    position1 = [0.0, 0.0]
    position2 = [0.0, 0.0]
    endTicks = 0.0
    slope = 0.0
    euclideanDistance = 0.0
    current_slope = 0.0
    current_angle = 0.0
    radius = 0.0
    distanceDriven = 0.0
    ratio = 0.0

subscribe_GPS = rospy.Subscriber("/communication/131/localization", Odometry, getGPS, queue_size=10)
publish_steering = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=100)
publish_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=100)
subscribe_tick = rospy.Subscriber("/sensors/arduino/ticks", Tick, countTicks, queue_size=100)


def test1():
    print("test 1")
    global x1, y1, position1, position2, endTicks
    rospy.sleep(3)
    position1 = [x1, y1]
    print("start position: " + str(position1))
    steering_command =  NormalizedSteeringCommand()
    steering_command.value = 1.0
    publish_steering.publish(steering_command)
    rospy.sleep(1)
    speed_command = SpeedCommand()
    speed_command.value = 0.2
    publish_speed.publish(speed_command)
    rospy.sleep(5)
    speed_command.value = 0.0
    publish_speed.publish(speed_command)
    position2 = [x1, y1]
    print("end position: " + str(position2))
    print("total ticks: " + str(endTicks)) 
    euclideanDistance = euclidean_distance(position1[0], position1[1], position2[0], position2[1])
    current_slope = calculate_slope(position1[0], position1[1], position2[0], position2[1])
    current_angle = math.atan(current_slope)
    print("current angle: " + str(current_angle))
    radius = (euclideanDistance/(2*math.sin(current_angle/2)))
    print("radius: " + str(radius))
    distanceDriven = radius*current_angle
    print("total distance driven: " + str(distanceDriven))
    ratio = distanceDriven/endTicks
    print("ratio between travelled distance and counted ticks: " + str(ratio))

def test2():
    print("test 2")
    global x1, y1, position1, position2, endTicks
    rospy.sleep(3)
    position1 = [x1, y1]
    print("start position: " + str(position1))
    steering_command =  NormalizedSteeringCommand()
    steering_command.value = 0.0
    publish_steering.publish(steering_command)
    rospy.sleep(1)
    speed_command = SpeedCommand()
    speed_command.value = 0.2
    publish_speed.publish(speed_command)
    rospy.sleep(5)
    speed_command.value = 0.0
    publish_speed.publish(speed_command)
    position2 = [x1, y1]
    print("end position: " + str(position2))
    print("total ticks: " + str(endTicks)) 
    euclideanDistance = euclidean_distance(position1[0], position1[1], position2[0], position2[1])
    current_slope = calculate_slope(position2[1], position1[1], position2[0], position1[0])
    current_angle = math.atan(current_slope)
    print("current angle: " + str(current_angle))
    radius = (euclideanDistance/(2*math.sin(current_angle/2)))
    print("radius: " + str(radius))
    distanceDriven = radius*current_angle
    print("total distance driven: " + str(distanceDriven))
    ratio = distanceDriven/endTicks
    print("ratio between travelled distance and counted ticks: " + str(ratio))

def test3():
    print("test 3")
    global x1, y1, position1, position2, endTicks
    rospy.sleep(3)
    position1 = [x1, y1]
    print("start position: " + str(position1))
    steering_command =  NormalizedSteeringCommand()
    steering_command.value = -1.0
    publish_steering.publish(steering_command)
    rospy.sleep(1)
    speed_command = SpeedCommand()
    speed_command.value = 0.2
    publish_speed.publish(speed_command)
    rospy.sleep(5)
    speed_command.value = 0.0
    publish_speed.publish(speed_command)
    position2 = [x1, y1]
    print("end position: " + str(position2))
    print("total ticks: " + str(endTicks)) 
    euclideanDistance = euclidean_distance(position1[0], position1[1], position2[0], position2[1])
    current_slope = calculate_slope(position2[1], position1[1], position2[0], position1[0])
    current_angle = math.atan(current_slope)
    print("current angle: " + str(current_angle))
    radius = (euclideanDistance/(2*math.sin(current_angle/2)))
    print("radius: " + str(radius))
    distanceDriven = radius*current_angle
    print("total distance driven: " + str(distanceDriven))
    ratio = distanceDriven/endTicks
    print("ratio between travelled distance and counted ticks: " + str(ratio))


test1()
resetVariables()
rospy.sleep(5)
test2()
resetVariables()
rospy.sleep(5)
test3()

#communication/gps/(markernumber)