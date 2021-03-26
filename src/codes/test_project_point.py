#!/usr/bin/env python
import roslib
import sys
import rospy
#import pcl
import numpy as np
import ctypes
import struct
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as im

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import tensorflow as tf
from tensorflow import keras
from sklearn.model_selection import train_test_split
import numpy as np
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import time
from numpy import savetxt
from numpy import save
from numpy import load

global z
z=0

def main(args):

    cloud_points = load('data_points.npy')
    fig =None
    if fig == None:
        plt.ion()
        plt.show()
        fig = plt.figure()
    while(True):
        run(cloud_points, fig)

def rotX(cloud, theta):

    cos = np.cos(theta)
    sin = np.sin(theta)
    MatrixX = np.array([[1   ,0   ,0  , 0],
                        [0   ,cos,-sin, 0],
                        [0   ,-sin, cos,0],
                        [0   ,0   ,0   ,1]])
    return MatrixX.dot(cloud.T).T

def rotY(cloud, theta):

    cos = np.cos(theta)
    sin = np.sin(theta)

    MatrixY = np.array([[cos ,0   ,sin ,0],
                        [0   ,1   ,0   ,0],
                        [-sin,0   ,cos ,0],
                        [0   ,0   ,0   ,1]])

    return MatrixY.dot(cloud.T).T

def rotZ(cloud, theta):

    cos = np.cos(theta)
    sin = np.sin(theta)

    MatrixZ = np.array([[cos ,-sin,0   ,0],
                        [sin ,cos ,0   ,0],
                        [0   ,0   ,1   ,0],
                        [0   ,0   ,0   ,1]])

    return MatrixZ.dot(cloud.T).T

def tran(cloud, vec):


    MatrixT = np.array([[1   ,0   ,0   ,vec[0]],
                        [0   ,1   ,0   ,vec[1]],
                        [0   ,0   ,1   ,vec[2]],
                        [0   ,0   ,0   ,1     ]])

    return MatrixT.dot(cloud.T).T

def pinhole(cloud):

    d=1
    width = 128 * 1
    height = 72 * 1
    MatrixP = np.array([[width   ,0      ,width/2  ,0 ],
                        [0       ,height ,height/2 ,0 ],
                        [0       ,0      ,-1/d     ,0 ]])

    return MatrixP.dot(cloud.T).T

def run(cloud_points, fig):
    global z

    width = 128
    height = 72

    z -= 0.1
    theta = np.pi * (0.5)
    cloud_points = rotY(cloud_points, theta)

    theta = np.pi * (0.5)
    cloud_points = rotZ(cloud_points, theta)

    vec =[0,-1,-z]

    cloud_points = tran(cloud_points, vec)

    cloud_points = pinhole(cloud_points)
    cloud_points = cloud_points.T

    indextoremove = []
    for i in range(len(cloud_points[2])):
        #print(i)
        if cloud_points[2][i] < 1:
            indextoremove.append(i)
        if cloud_points[2][i] > 6:
            indextoremove.append(i)

    cloud_points = np.delete(cloud_points,indextoremove,1)


    xy = np.array([np.around(cloud_points[0]/cloud_points[2] + width ),
                   np.around(cloud_points[1]/cloud_points[2] + height),
                   cloud_points[2] ])

    uv = np.ones((height, width, 1))*6

    xy = xy.T

    for point in xy:
        if point[0] < width and point[0] >= 0 and point[1] < height and point[1] >= 0:
            v = int(point[0])
            u = int(point[1])
            if point[2] < uv[u][v][0]:
                uv[u][v][0] = point[2]

    uv[uv >= 6] = 0

    plt.gca().invert_yaxis()
    img = np.float32(np.reshape(uv,(72,128)))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(4,4))
    img = cv2.dilate(img,kernel,iterations = 1)
    img = cv2.erode(img,kernel,iterations = 1)

    plt.imshow(img, origin='lower' )
    plt.pause(0.001)
    plt.cla()

    pass

if __name__ == '__main__':
    main(sys.argv)
