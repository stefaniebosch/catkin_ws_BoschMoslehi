import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

DF = pd.read_csv("C:/Users/sinam/Desktop/file.csv")
DF.drop(columns=['%time', 'field.header.seq', 'field.header.stamp',
                 'field.header.frame_id'],inplace=True)
DF.rename(columns={"field.K0":"fx","field.K2":"cx","field.K4":"fy",
                   "field.K5":"cy","field.D0":"k1",
                   "field.D1":"k2","field.D2":"t1","field.D3":"t2",
                   "field.D4":"k3"}, inplace= True)
parameter = DF[["fx", "fy", "cx","cy", "k1", "k2", "t1", "t2", "k3"]]

print (parameter.iloc[0] )

img = cv2.imread("D:/Uni/Robotik/Uebung3/Image/frame0040.jpg")
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
retval, threshold = cv2.threshold(gray, 100, 255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
rows, cols = threshold.shape
cv2.rectangle(threshold, (0, 0), (cols, 107), (0,0,0), -1)
cv2.rectangle(threshold, (0, 0), (175, rows), (0,0,0), -1)
cv2.rectangle(threshold, (0, 246), (cols, rows), (0,0,0), -1)
cv2.rectangle(threshold, (250,210 ), (280, 230), (0,0,0), -1)
cv2.imshow("orginal",gray)
cv2.imshow("threshold", threshold)
cv2.imwrite("d:/Uni/Robotik/Uebung3/Image/threshold.jpg", threshold)
cv2.waitKey(0)

#divide the thresholded image into 6 regions
#calculate the average position to get the center of the marking
y1 = 0
y2 = 140
x1 = 0
x2 = int(threshold.shape[1]/2)
count = 0
points = np.empty(0)
while count != 6:
    reg = threshold[y1:y2 , x1:x2]
    indices = np.where(reg == [255])
    x = np.average(indices[1])+ x1
    y = np.average(indices[0])+ y1
    points = np.append(points,(x,y))
    count += 1
    if count == 2:
        y1 = 0
        y2 = 140
    reg = threshold[y1:y2 , x2:]
    indices = np.where(reg == [255])
    indices = np.where(reg == [255])
    x = np.average(indices[1])+ x2
    y = np.average(indices[0])+ y1
    points = np.append(points,(x,y))
    y1 = y2
    y2 += 50
    count += 1
    
average = points.reshape(6,2)
 #center of the marking in array
print (average)



fig=plt.figure(figsize=(30,30))
for i in range(1, len(average) + 1):
    plt.imshow(threshold, cmap="gray")
    plt.scatter(average[i-1][0], average[i-1][1], s=10, c='red', marker='o')
plt.show()

parameterNP = np.array(parameter.iloc[0]) # change to numpy array
def matrixK(parameter):    
    K = np.zeros(9)
    for i in range(K.shape[0]):
        if i == 0:
            K[i] = parameterNP[0]  # fx
        elif i == 2:
            K[i] = parameterNP[2]  # cx
        elif i == 4:
            K[i] = parameterNP[1]  # fy
        elif i == 5:
            K[i] = parameterNP[3]  # cy
        elif i == 8:
            K[i] = 1
        else :
            K[i] = 0
    K = K.reshape(3,3)
    return K
	
	
#a 3x3 numpy array for the intrinsic parameters
print(matrixK(parameterNP))

# a 3x3 numpy array for the intrinsic parameters
distParameters = np.array(parameterNP[4:])
print(distParameters)

objectPoints = np.array([[0.5, 0.2,0.0],
                         [0.5, -0.2,0.0],
                         [0.8, 0.2,0.0],
                         [0.8, -0.2,0.0],
                         [1.1, 0.2,0.0],
                         [1.1, -0.2,0.0]])
imagePoints = average
cameraMatrix = matrixK(parameterNP)

ret, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints,
                               cameraMatrix, distParameters)
							   
print("rotation cevtor: \n",rvec) # Output rotation vector
print("translation cevtor: \n",tvec) # Output translation vector

# convert to a 3x3 rotation matrix
rotationMatrix = cv2.Rodrigues(rvec)
print (rotationMatrix[0])

# homogeneous 4x4 transformation matrix
letzeReihe = np.array([0, 0, 0, 1])
homoTrans = np.concatenate((rotationMatrix[0], tvec),axis = 1)
homoTrans = np.vstack((homoTrans, letzeReihe))
homoTrans

# camera, transform matrix in relation to the "world"
HomoTransInv = cv2.invert(homoTrans)
print(HomoTransInv[1])








