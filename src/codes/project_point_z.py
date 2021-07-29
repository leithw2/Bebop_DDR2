#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge, CvBridgeError


import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

import matplotlib.pyplot as plt
import numpy as np
import matplotlib as mpl
from numpy import save
from numpy import load

class pointCloud:

    def __init__(self):
        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("/rtabmap/cloud_map",PointCloud2,self.callback)
        self.image_sub = rospy.Subscriber("/orb_slam2_mono/map_points",PointCloud2,self.callback)
        self.drone_image_pub = rospy.Publisher("/bebop/image_map",Image, queue_size=1)
        self.fig = None
        self.z=0
        self.width =1080
        self.height=1080


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
        width = self.width
        height = self.height
        MatrixP = np.array([[width   ,0      ,width/2  ,0 ],
                            [0       ,width  ,height/2  ,0 ],
                            [0       ,0      ,-1/d     ,0 ]])

        return MatrixP.dot(cloud.T).T

    def ortho(self, cloud):

        d=1
        width = self.width
        height = self.height
        MatrixP = np.array([[1       ,0      ,width/2  ,0 ],
                            [0       ,1      ,height/2 ,0 ],
                            [0       ,0      ,0        ,1]])

        return MatrixP.dot(cloud.T).T

    def eucDist(self, point):
        return np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)

    def send_image_map(self, image):
        img = image.astype(np.uint8)

        image_message = self.bridge.cv2_to_imgmsg(img, "mono8")
        try:
            self.drone_image_pub.publish(image_message)
        except rospy.ROSInterruptException as ros_e :
            print(ros_e)

        #self.rate.sleep()

    def callback(self,data):
        cloud_points = list(pc2.read_points(data, skip_nans=True, field_names = ("x", "y", "z" )))
        cloud_points = np.array(cloud_points)
        cloud_points = np.insert(cloud_points, 3, 1, axis = 1)
        save('data_points_ddr.npy', cloud_points)
        #cloud_points = load('data_points_ddr.npy')
        width = self.width
        height = self.height

        self.z = -1.5
        theta = np.pi * (0.5)
        cloud_points = self.rotY(cloud_points, theta)

        theta = np.pi * (0.5)
        cloud_points = self.rotZ(cloud_points, theta)

        meanx = np.mean(cloud_points.T[0])
        meany = np.mean(cloud_points.T[1])
        #print(cloud_points[0])
        #print(meanx, meany)
        vec =[-meanx,-meany,self.z] #
        #vec =[0,0.0,self.z] #

        cloud_points = self.tran(cloud_points, vec)

        cloud_points = self.pinhole(cloud_points)
        #cloud_points = self.ortho(cloud_points)
        cloud_points = cloud_points.T

        indextoremove = []
        # for i in range(len(cloud_points[2])):
        #     print(i)
        #     if cloud_points[2][i] < -5:
        #         indextoremove.append(i)
        #     if cloud_points[2][i] > 5:
        #         indextoremove.append(i)

        cloud_points = np.delete(cloud_points,indextoremove,1)


        xy = np.array([np.around(cloud_points[0]/cloud_points[2] + width ),
                       np.around(cloud_points[1]/cloud_points[2] + height),
                       cloud_points[2] ])

        #xy = cloud_points
        indextoremove =[]
        # for i in range(len(xy[2])):
        #     #print(i)
        #     if xy[2][i] < 4:
        #         indextoremove.append(i)
        #     if xy[2][i] > 5:
        #         indextoremove.append(i)

        xy = np.delete(xy,indextoremove,1)

        uv = np.ones((height, width, 1))*100
        uvpre = np.ones((height, width, 1))*100

        xy = xy.T

        for point in xy:
            if point[0] < width and point[0] >= 0 and point[1] < height and point[1] > 0:
                v = int(point[0])
                u = int(point[1])
                if uv[u,v,0] > point[2]:
                    uv[u,v,0] = point[2]

        #print(np.mean(xy.T[2]))

        mean = np.mean(xy.T[2])

        #print(mean - (mean *  .8))
        #print(mean + (mean *  .05))
        #print("")

        uv[:,:,0][uv[:,:,0] >= mean + (mean *  .05)] = 0
        uv[:,:,0][uv[:,:,0] < mean - (mean *  .9) ] = 0
        uv[:,:,0][uv[:,:,0] != 0 ] = 255

        uv = np.flip(uv,0)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10,10))
        uv = cv2.dilate(uv,kernel,iterations = 2)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        uv = cv2.erode(uv,kernel,iterations = 2)

        self.send_image_map(uv)

        #plt.imshow(np.reshape(uv,(height,width)),origin='lower')
        #plt.pause(.001)
        #plt.cla()
        #print(np.array(data.data).shape)
        pass


def main(args):

    print("Starting pointCloud Node ...")
    rospy.init_node('pointCloud_Node', anonymous=True)
    pc = pointCloud()

    try:
        while True:
            #pc.callback("")
            pass
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
