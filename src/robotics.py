#Da wir Aufgabe 7-2 nicht ganz geschafft haben, hoffen wir das wir Teilpunkte kriegen
#koennen basierend auf das was wir probiert haben.

import cv2
import matplotlib.pyplot as plt
import numpy as np
%matplotlib inline

image = cv2.imread("C:/Users/sinam/Desktop/auto.jpg",0)
def removeWhitePoints(image):
    retval, threshold = cv2.threshold(image, 100, 255,
                                  cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    rows, cols = threshold.shape
    cv2.rectangle(threshold, (0, 0), (cols, 90), (0,0,0), -1)
    cv2.rectangle(threshold, (190, 260), (525, rows), (0,0,0), -1)
    vertices = np.array([ [400, 100], [cols, 100], [cols, 300]])
    pts = vertices.reshape((-1,1,2))
    cv2.fillPoly(threshold, [pts], color = (0,255,0))
    
    return threshold
    

removeWhitePoints(image)

plt.imshow(removeWhitePoints(image),cmap="gray")

import numpy as np
import scipy
import matplotlib.pyplot as plt
import math
import sys
import cv2
import random

image = cv2.imread('C:/Users/sinam/Desktop/auto.jpg',0)
picturePoints = []

#initializations

samplePoints = []
t_distanceThreshold = 3
#am Anfang ist n_iterations 'unendlich'
n_iterations = 1000000
sampleCount = 0
inlierRatio = 0.5
outlierRatio = 0.5
p = 0.95
numInliers = 0
y1=0
y2=image.shape[1]
x1=0
x2=image.shape[0]

picturePoints.size

#turns unecessary white points black
retval, threshold = cv2.threshold(image, 100, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
rows, cols = threshold.shape
cv2.rectangle(threshold, (0, 0), (cols, 90), (0,0,0), -1)
cv2.rectangle(threshold, (190, 260), (525, rows), (0,0,0), -1)
vertices = np.array([ [400, 100], [cols, 100], [cols, 300]])
pts = vertices.reshape((-1,1,2))
cv2.fillPoly(threshold, [pts], color = (0,255,0))
reg = threshold[y1:y2 , x1:x2]
position = np.where(reg == [255])
np.append(picturePoints, (position[0], position[1]))
picturePoints = np.dstack((position[0],position[1]))

picturePoints.shape

totalPoints = picturePoints.size

#calculates the line to be drawn
def line(points1, points2):
    m = ((points2[1]-points1[1])/(points2[0]-points1[0]))
    n = points2[1] - m * points2[0]
    return m, n

#calculates the distance between two points
def dist(x0, y0, x1, y1):
    return math.sqrt((x1 - x0)**2 + (y1 - y0)**2)

#counts how many inliers there are
def countInliers(x, y):
    global numInliers
    if dist(x, y)<t_distanceThreshold:
        numInliers += 1
    return numInliers

sampleCount = 0
print(n_iterations)
print(sampleCount)
#like in the lecture slides
while n_iterations>sampleCount:
    #takes 2 random points
    point1 = samplePoints.append(random.choice(picturePoints))
    point2 = samplePoints.append(random.choice(picturePoints))
    e_inlierRatio = 1-(numInliers/totalPoints)
    n_iterations = (math.log(1-p))/(math.log(1-(1-inlierRatio)**2))
    sampleCount += 1
    #counts how many inliers there are
    for point in samplePoints:
        countInliers(point[0], point[1])
    #if best fitting line, overwrite old line
    if (numInliers/float(totalPoints))>inlierRatio:
        inlierRatio = numInliers/float(totalPoints)
        m, n = line(point1, point2)
        bestStartPoint = point1
        bestEndPoint = point2
        line_m = m
        line_n = n

#draw and publish line
newImage = cv2.line(image, bestStartPoint, bestEndPoint, (255, 0, 0), 5)
cv2.imshow('image', newImage)