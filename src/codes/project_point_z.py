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
from geometry_msgs.msg import PoseStamped, Point

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
        self.image_sub      = rospy.Subscriber("/orb_slam2_mono/map_points",PointCloud2,self.callback)
        self.pose_sub       = rospy.Subscriber("/orb_slam2_mono/pose",PoseStamped,self.callback_pose)
        self.bebop_state_sub= rospy.Subscriber('/bebop/state', String, self.bebop_state_callback)

        self.image_pub      = rospy.Publisher("/bebop/image_map",Image, queue_size=1)
        self.start_pub      = rospy.Publisher("/bebop/start_map",Point, queue_size=1)
        self.goal_pub       = rospy.Publisher("/bebop/goal_map",Point, queue_size=1)

        self.fig = None
        self.z=0
        self.width =1080
        self.height=1080
        self.pose = []
        self.bebop_state = 0
        self.goalx = 0
        self.goaly = 0
        self.goalz = 0
        self.startx = 0
        self.starty = 0
        self.startz = 0



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
            self.image_pub.publish(image_message)
        except rospy.ROSInterruptException as ros_e :
            print(ros_e)

        #self.rate.sleep()
    def send_start(self, p):

        #self.rate.sleep()
        point   = Point()
        point.x = p[0]
        point.y = p[1]
        point.z = 0

        try:
            self.start_pub.publish(point)
        except rospy.ROSInterruptException as ros_e :
            print(ros_e)

    def send_goal(self, p):

        #self.rate.sleep()
        point   = Point()
        point.x = p[0]
        point.y = p[1]
        point.z = 0

        try:
            self.goal_pub.publish(point)
        except rospy.ROSInterruptException as ros_e :
            print(ros_e)

    def bebop_state_callback(self, data):
        self.bebop_state = int(data.data)
        #print(self.goalx,self.goaly)

    def callback_pose(self,data):
        self.pose = data.pose

        #print(self.bebop_state)
        if self.bebop_state == 4:
            self.goalx = 1
            self.goaly = self.pose.position.y
            self.goalz = self.pose.position.z
            #print(data.pose)

        if self.bebop_state == 5 and self.goalx == 0 and self.goaly == 0:
            self.goalx = 1
            self.goaly = self.pose.position.y
            self.goalz = self.pose.position.z
            #print(data.pose)

        self.startx = 1
        self.starty = self.pose.position.y
        self.startz = self.pose.position.z

    def callback(self,data):

        if self.goalx == 0:
            return


        cloud_points = list(pc2.read_points(data, skip_nans=True, field_names = ("x", "y", "z" )))
        cloud_points = np.array(cloud_points)
        cloud_points = np.insert(cloud_points, 3, 1, axis = 1)
        save('data_points_ddr.npy', cloud_points)
        #cloud_points = load('data_points_ddr.npy')

        goal  = np.array([[self.goalx ,self.goaly ,self.goalz,1]])
        start = np.array([[self.startx,self.starty,self.startz,1]])

        width = self.width
        height = self.height

        self.z = -1.5
        theta = np.pi * (0.5)

        cloud_points = self.rotY(cloud_points, theta)
        goal = self.rotY(goal, theta)
        start = self.rotY(start, theta)

        theta = np.pi * (0.5)
        cloud_points = self.rotZ(cloud_points, theta)
        goal = self.rotZ(goal, theta)
        start = self.rotZ(start, theta)

        meanx = np.mean(cloud_points.T[0])
        meany = np.mean(cloud_points.T[1])
        meanz = np.mean(cloud_points.T[2])

        #print(cloud_points[0])
        #print(meanz)
        vec =[-meanx,-meany,self.z] #
        #vec =[0,0.0,self.z] #

        cloud_points = self.tran(cloud_points, vec)
        goal = self.tran(goal, vec)
        start = self.tran(start, vec)

        cloud_points = self.pinhole(cloud_points)
        goal = self.pinhole(goal)
        start = self.pinhole(start)

        #cloud_points = self.ortho(cloud_points)
        cloud_points = cloud_points.T
        goal = goal.T
        start = start.T
        #print(goal)

        indextoremove = []
        # for i in range(len(cloud_points[2])):
        #     print(i)
        #     if cloud_points[2][i] < -5:
        #         indextoremove.append(i)
        #     if cloud_points[2][i] > 5:
        #         indextoremove.append(i)

        #cloud_points = np.delete(cloud_points,indextoremove,1)


        xy = np.array([np.around(cloud_points[0]/cloud_points[2] + width ),
                       np.around(cloud_points[1]/cloud_points[2] + height),
                       cloud_points[2] ])

        goal_image = np.array([np.around(goal[0]/goal[2] + width ),
                       np.around(goal[1]/goal[2] + height),
                       goal[2] ])

        start_image = np.array([np.around(start[0]/start[2] + width ),
                       np.around(start[1]/start[2] + height),
                       start[2] ])

        #print(goal_image.T)

        #print(int(start_image[0]))
        #xy = cloud_points
        indextoremove =[]
        # for i in range(len(xy[2])):
        #     #print(i)
        #     if xy[2][i] < 4:
        #         indextoremove.append(i)
        #     if xy[2][i] > 5:
        #         indextoremove.append(i)

        #xy = np.delete(xy,indextoremove,1)
        uv = np.ones((height, width, 1))*100
        xy = xy.T

        for point in xy:
            if point[0] < width and point[0] >= 0 and point[1] < height and point[1] > 0:
                v = int(point[0])
                u = int(point[1])
                if uv[u,v,0] > point[2]:
                    uv[u,v,0] = point[2]
                    # print("points draw")

        #print(np.mean(xy.T[2]))

        mean = np.mean(xy.T[2])

        #print(mean - (mean *  .8))
        #print(mean + (mean *  .05))
        #print("")

        uv[:,:,0][uv[:,:,0] >= mean + (mean *  .05)] = 0
        uv[:,:,0][uv[:,:,0] < mean - (mean *  .9) ] = 0
        uv[:,:,0][uv[:,:,0] != 0 ] = 255



        uv = np.flip(uv,0)
        start_image[0] = int(start_image[0])
        start_image[1] = int(1080 - start_image[1])
        goal_image[0]  = int(goal_image[0])
        goal_image[1]  = int(1080 - goal_image[1])

        # uv[int(start_image[0])][int(start_image[1])] = 2000
        # uv[int(goal_image[0] )][int(goal_image[1] )] = 1000

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10,10))
        uv = cv2.dilate(uv,kernel,iterations = 2)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        uv = cv2.erode(uv,kernel,iterations = 2)



        # print(uv[int(start_image[0])][int(start_image[1])])
        # print(start_image[0], start_image[1])

        self.send_goal(goal_image)
        self.send_start(start_image)
        self.send_image_map(uv)

        # plt.imshow(np.reshape(uv,(height,width)),origin='lower')
        # plt.pause(.001)
        # plt.cla()
        #print(np.array(data.data).shape)

        pass


def main(args):

    print("Starting pointCloud Node ...")
    rospy.init_node('pointCloud_Node', anonymous=True)
    pc = pointCloud()

    try:
        # while True:
        #     pc.callback("")
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
