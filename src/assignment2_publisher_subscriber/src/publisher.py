#!/usr/bin/env python

import rospy
from autominy_msgs.msg import NormalizedSteeringCommand
from autominy_msgs.msg import SpeedCommand

#Initialize node
rospy.init_node("rplidar_driver_node")

#Initialize publisher
publisher=rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand)
publisher2=rospy.Publisher("/actuators/speed", SpeedCommand)
 
while not rospy.is_shutdown():
    msg=NormalizedSteeringCommand()
    msg.value=1.0
    publisher.publish(msg)
    msg2=SpeedCommand()
    msg2.value=0.3
    publisher2.publish(msg2)
    rospy.sleep(0.5)
