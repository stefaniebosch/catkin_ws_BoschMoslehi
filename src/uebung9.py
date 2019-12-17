#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from autominy_msgs.msg import SpeedCommand
from autominy_msgs.msg import NormalizedSteeringCommand

rospy.init_node("uebung9", anonymous=True)

kP=1.0
kD=0.0
currAngle=0.0
desiredAngle=0.0
speed_command=SpeedCommand()
steering_command=NormalizedSteeringCommand()
oldTime=rospy.get_time()
newTime=rospy.get_time()
oldError=0.0

yawAngle=input("Please input desired yaw angle: 0 for 0, 1 for pi, 2 for -pi: ")

if yawAngle==0:
    desiredAngle=0
elif yawAngle==1:
    desiredAngle=math.pi
else:
    desiredAngle=-1*(math.pi)

def getOdometry(odometry):
    global currAngle
    currAngle=math.asin(odometry.pose.pose.orientation.z)*2
    print("currAngle: ", currAngle)

subscribe_odometry = rospy.Subscriber("/communication/121/localization", Odometry, getOdometry, queue_size=10)
publish_steering = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=100)
publish_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=100)

def pdController(angle):
    while(1):
        global currAngle, desiredAngle, oldTime, newTime, oldError, kP, kD
        desiredAngle=angle
        currentAngle=currAngle
        #constant speed of 0.2
        speed_command.value = 0.2
        publish_speed.publish(speed_command)
        #calculates current angle
        newTime=rospy.get_time()
        newError=desiredAngle-currentAngle
        #proportional part
        pResult=kP*newError
        print("pResult: ", pResult)
        newTime=rospy.get_time()
        deltaT=newTime-oldTime
        #derivative part
        dResult=kD*((newError-oldError)/deltaT)
        print("dResult: ", dResult)
        oldError=newError
        oldTime=newTime
        #adds p and d parts together
        totalResult=pResult+dResult
        print("totalResult: ", totalResult)
        #calculates whether the car should steer left or right
        #takes total result and caps it to either 1 or negative 1 based on total result
        if totalResult>0.0:
            steering_command.value=1.0
        elif totalResult<0.0:
            steering_command.value=-1.0
        else:
            steering_command.value=0.0

pdController(desiredAngle)