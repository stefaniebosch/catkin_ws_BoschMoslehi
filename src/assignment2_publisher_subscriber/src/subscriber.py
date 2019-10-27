#!/usr/bin/env python

import rospy
#from rplidar_decoder import PointCloudConverter
from autominy_msgs.msg import Speed
 
def callback(raw_msg):
    rospy.loginfo(raw_msg)

rospy.init_node("subscriber", anonymous=True)

rospy.Subscriber("/sensors/speed", Speed, callback)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
 
