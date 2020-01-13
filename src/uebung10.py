#!/usr/bin/env python

import rospy
import math
#python -m pip install --user scipy
import numpy
from scipy.interpolate import CubicSpline
from numpy.linalg import norm
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
import matplotlib.pyplot as plt

cx=0.0
cy=0.0

def getClickedPoint(c_point):
    global cx, cy
    cx=c_point.point.x
    cy=c_point.point.y
    print("clicked x: {} clicked y: {}".format(cx, cy))

#publisher
rospy.init_node('splinepub', anonymous=True)
publish_spline = rospy.Publisher("visualization_msgs/Marker", Marker, queue_size=100)
subscribe_clickedPoint = rospy.Subscriber("/clicked_point", PointStamped, getClickedPoint, queue_size=100)


####################10-1#####################################
x1 = numpy.arange(0.0, 12.805, 0.1)

lane1=numpy.load("lane1.npy")

#arrays that save the values from the first lane
saved1Values=[]
x1Values=[]
y1Values=[]
arc1Values=[]

#save 20 values, so take every 65 values; 1277 length of lane1
for x in range(0, 1277, 65):
    saved1Values.append(lane1[x])

for item in saved1Values:
    arc1Values.append(item[0])
    x1Values.append(item[1])
    y1Values.append(item[2])

print("#################### now for lane 1 ###########################")
print("")
print("arc1 values:")
print(arc1Values)
print("")
print("x1 values:")
print(x1Values)
print("")
print("y1 values:")
print(y1Values)
print("")

splinex1=CubicSpline(arc1Values, x1Values)
spliney1=CubicSpline(arc1Values, y1Values)

plt.plot(splinex1(x1), spliney1(x1), label='spline lane 1')

print("#################### now for lane 2 ###########################")
print("")

x2 = numpy.arange(0.0, 14.805, 0.1)

lane2=numpy.load("lane2.npy")

#arrays that save the values from the second lane
saved2Values=[]
x2Values=[]
y2Values=[]
arc2Values=[]

#save 20 values, so take every 74 values; 1477 length of lane2
for x in range(0, 1477, 74):
    saved2Values.append(lane2[x])

for item in saved2Values:
    arc2Values.append(item[0])
    x2Values.append(item[1])
    y2Values.append(item[2])

print("arc2 values:")
print(arc2Values)
print("")
print("x2 values:")
print(x2Values)
print("")
print("y2 values:")
print(y2Values)
print("")

splinex2=CubicSpline(arc2Values, x2Values)
spliney2=CubicSpline(arc2Values, y2Values)

plt.plot(splinex2(x2), spliney2(x2), label='spline y1')

plt.plot(x1Values, y1Values, 'ro', x2Values, y2Values, 'bo')
#plt.show()

##################10-2###################
######binary search for closest point#######

splinex2values=[]
spliney2values=[]
for value in x2:
    splinex2values.append(splinex2(value))
    spliney2values.append(spliney2(value))

samplePoints=[]
for num in range(len(splinex2values)):
    samplePoints.append([float(splinex2values[num]), float(spliney2values[num])]) 

print("########################Now showing spline in rviz##########################")

while not rospy.is_shutdown():

    ###################10-1############################
    #initialisation for marker1: spline1
    marker1=Marker()
    marker1.type = marker1.LINE_STRIP
    marker1.action = marker1.ADD
    marker1.header.frame_id = "map"
    marker1.header.stamp = rospy.Time()
    marker1.ns = "marker_1"
    marker1.id = 1
    marker1.scale.x = 0.03
    marker1.pose.position.x = 0.0
    marker1.pose.position.y = 0.0
    marker1.pose.position.z = 0.0
    marker1.pose.orientation.x = 0.0
    marker1.pose.orientation.y = 0.0
    marker1.pose.orientation.z = 0.0
    marker1.pose.orientation.w = 1.0
    marker1.color.a = 1.0
    marker1.color.r = 0.0
    marker1.color.g = 1.0
    marker1.color.b = 0.0

    #initialisation for marker2: spline2
    marker2=Marker()
    marker2.type = marker2.LINE_STRIP
    marker2.action = marker2.ADD
    marker2.header.frame_id = "map"
    marker2.header.stamp = rospy.Time()
    marker2.ns = "marker_2"
    marker2.id = 2
    marker2.scale.x = 0.03
    marker2.pose.position.x = 0.0
    marker2.pose.position.y = 0.0
    marker2.pose.position.z = 0.0
    marker2.pose.orientation.x = 0.0
    marker2.pose.orientation.y = 0.0
    marker2.pose.orientation.z = 0.0
    marker2.pose.orientation.w = 1.0
    marker2.color.a = 1.0
    marker2.color.r = 1.0
    marker2.color.g = 0.0
    marker2.color.b = 0.0

    #set points for marker1
    marker1.points = []
    for value in x1:
        first_spline_point=Point()
        first_spline_point.x = splinex1(value)
        first_spline_point.y = spliney1(value)
        first_spline_point.z = 0.0
        marker1.points.append(first_spline_point)
    
    #set points for marker2
    marker2.points = []
    for value in x2:
        second_spline_point=Point()
        second_spline_point.x = splinex2(value)
        second_spline_point.y = spliney2(value)
        second_spline_point.z = 0.0
        marker2.points.append(second_spline_point)

    ##############################10-2###############################
    #clickedpoint
    marker3=Marker()
    marker3.type = marker3.SPHERE
    marker3.action = marker3.ADD
    marker3.header.frame_id = "map"
    marker3.header.stamp = rospy.Time()
    marker3.id = 3
    marker3.ns = "marker_3"
    marker3.scale.x=.25
    marker3.scale.y=.25
    marker3.scale.z=.25
    marker3.pose.position.x = cx
    marker3.pose.position.y = cy
    marker3.pose.position.z = 0.0
    marker3.pose.orientation.x = 0.0
    marker3.pose.orientation.y = 0.0
    marker3.pose.orientation.z = 0.0
    marker3.pose.orientation.w = 1.0
    marker3.color.a = 1.0
    marker3.color.r = 0.0
    marker3.color.g = 0.0
    marker3.color.b = 1.0

    #################################10-2#####################################
    step_size=0.2
    binaryTestPoints=[]
    minimum=0.0
    maximum=len(samplePoints)
    closestArc=0.0
    wanted_precision=0.0001
    newDistance=100.0

    while(step_size>wanted_precision):
        closestDistance=100.0
        #creates sample values
        wanted_steps = numpy.arange(minimum, maximum, step_size)
        for num in range(len(wanted_steps)):
            #plug in values into spline
            binaryTestPoints.append([float(splinex2(wanted_steps[num])), float(spliney2(wanted_steps[num]))])
        #calculates closest point to clicked point from list of points
        for num in range(len(binaryTestPoints)):
            newDistance=norm([binaryTestPoints[num][0]-cx, binaryTestPoints[num][1]-cy])
            #finds smallest distance
            if(newDistance<closestDistance):
                closestDistance=newDistance
                closestx=binaryTestPoints[num][0]
                closesty=binaryTestPoints[num][1]
                closestArc=wanted_steps[num]
                
                ##################10-3####################
                lookaheadx=float(splinex2(wanted_steps[num]+2))
                lookaheady=float(spliney2(wanted_steps[num]+2))
                
        #takes neighboring points
        minimum = closestArc - step_size
        maximum = closestArc + step_size
        #lower step size
        step_size *= 0.5
        #clear list
        del binaryTestPoints[:]

    #closestpoint
    marker4=Marker()
    marker4.type = marker4.SPHERE
    marker4.action = marker4.ADD
    marker4.header.frame_id = "map"
    marker4.header.stamp = rospy.Time()
    marker4.id = 4
    marker4.ns = "marker_4"
    marker4.scale.x=.25
    marker4.scale.y=.25
    marker4.scale.z=.25
    marker4.pose.position.x = closestx
    marker4.pose.position.y = closesty
    marker4.pose.position.z = 0.0
    marker4.pose.orientation.x = 0.0
    marker4.pose.orientation.y = 0.0
    marker4.pose.orientation.z = 0.0
    marker4.pose.orientation.w = 1.0
    marker4.color.a = 1.0
    marker4.color.r = 1.0
    marker4.color.g = 0.0
    marker4.color.b = 1.0

    ################10-3####################
    #lookaheadpoint
    marker5=Marker()
    marker5.type = marker5.SPHERE
    marker5.action = marker5.ADD
    marker5.header.frame_id = "map"
    marker5.header.stamp = rospy.Time()
    marker5.id = 5
    marker5.ns = "marker_5"
    marker5.scale.x=.25
    marker5.scale.y=.25
    marker5.scale.z=.25
    marker5.pose.position.x = lookaheadx
    marker5.pose.position.y = lookaheady
    marker5.pose.position.z = 0.0
    marker5.pose.orientation.x = 0.0
    marker5.pose.orientation.y = 0.0
    marker5.pose.orientation.z = 0.0
    marker5.pose.orientation.w = 1.0
    marker5.color.a = 1.0
    marker5.color.r = 0.0
    marker5.color.g = 1.0
    marker5.color.b = 0.0

    #publish both markers to rviz
    publish_spline.publish(marker1)
    publish_spline.publish(marker2)
    publish_spline.publish(marker3)
    publish_spline.publish(marker4)
    publish_spline.publish(marker5)