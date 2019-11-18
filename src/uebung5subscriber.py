#!/usr/bin/env python

import math
import rospy
from autominy_msgs.msg import SteeringAngle
 
def callback(raw_msg):
    #gives the steering angle in radians
    rospy.loginfo(math.radians(raw_msg))

rospy.init_node("subscriber", anonymous=True)

#subscribed to /sensors/arduino(steering_angle)
rospy.Subscriber("/sensors/arduino/steering_angle", SteeringAngle, callback)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()