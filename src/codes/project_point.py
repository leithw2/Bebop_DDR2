#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

import matplotlib.pyplot as plt
import numpy as np
import matplotlib as mpl


class pointCloud:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/rtabmap/cloud_map",PointCloud2,self.callback)
        self.fig = None
        self.z=0

    def rotX(self, cloud, theta):

        cos = np.cos(theta)
        sin = np.sin(theta)
        MatrixX = np.array([[1   ,0   ,0  , 0],
                            [0   ,cos,-sin, 0],
                            [0   ,-sin, cos,0],
                            [0   ,0   ,0   ,1]])
        return MatrixX.dot(cloud.T).T

    def rotY(self, cloud, theta):

        cos = np.cos(theta)
        sin = np.sin(theta)

        MatrixY = np.array([[cos ,0   ,sin ,0],
                            [0   ,1   ,0   ,0],
                            [-sin,0   ,cos ,0],
                            [0   ,0   ,0   ,1]])

        return MatrixY.dot(cloud.T).T

    def rotZ(self, cloud, theta):

        cos = np.cos(theta)
        sin = np.sin(theta)

        MatrixZ = np.array([[cos ,-sin,0   ,0],
                            [sin ,cos ,0   ,0],
                            [0   ,0   ,1   ,0],
                            [0   ,0   ,0   ,1]])

        return MatrixZ.dot(cloud.T).T

    def tran(self, cloud, vec):


        MatrixT = np.array([[1   ,0   ,0   ,vec[0]],
                            [0   ,1   ,0   ,vec[1]],
                            [0   ,0   ,1   ,vec[2]],
                            [0   ,0   ,0   ,1     ]])

        return MatrixT.dot(cloud.T).T

    def pinhole(self, cloud):

        d=1
        width = 128 * 1
        height = 72 * 1
        MatrixP = np.array([[width   ,0      ,width/2  ,0 ],
                            [0       ,width ,height/2 ,0 ],
                            [0       ,0      ,-1/d     ,0 ]])

        return MatrixP.dot(cloud.T).T

    def eucDist(self, point):
        return np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)

    def callback(self,data):

        cloud_points = list(pc2.read_points(data, skip_nans=True, field_names = ("x", "y", "z" )))
        cloud_points = np.array(cloud_points)
        cloud_points = np.insert(cloud_points, 3, 1, axis = 1)
        save('data_points.npy', cloud_points)
        width = 128
        height = 72

        self.z -= 0.1
        theta = np.pi * (0.5)
        cloud_points = self.rotY(cloud_points, theta)

        theta = np.pi * (0.5)
        cloud_points = self.rotZ(cloud_points, theta)

        vec =[0,-1,self.z]

        cloud_points = self.tran(cloud_points, vec)

        cloud_points = self.pinhole(cloud_points)
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

        plt.imshow(np.reshape(uv,(72,128)),origin='lower')
        plt.pause(0.001)
        plt.cla()
        #print(np.array(data.data).shape)
        pass


def main(args):
    pc = pointCloud()
    print("Starting pointCloud Node ...")
    rospy.init_node('pointCloud_Node', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
