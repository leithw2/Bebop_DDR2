#!/usr/bin/env python
import roslib
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt
from matplotlib import cm
from scipy.misc import imread
import random, sys, math, os
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Point, Pose, PoseStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from PIL import Image as im

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/RRTStar")

MAP_IMG = './maze.jpg' # Black and white image for a map

try:
    from rrt_star import RRTStar
except ImportError:
    raise

show_animation = True

class RRT_ROS:

    def __init__(self):

        self.Odometry = False
        self.rate = rospy.Rate(1)
        #########
        self.debug_image = imread(MAP_IMG,mode="L")
        print(type(self.debug_image ))
        ########

        self.bridge = CvBridge()
        self.fig = None
        # self.fig = plt.figure()
        # self.fig.clf()
        # self.ax = self.fig.add_subplot(1, 1, 1)

        self.image_sub = rospy.Subscriber("/rtabmap/grid_map",OccupancyGrid,self.callback)
        self.path_pub = rospy.Publisher("/rrt/path",Path, queue_size=1)
        self.path_image_pub = rospy.Publisher("/debug/image_path",Image, queue_size=1)
        self.odom_sub = rospy.Subscriber("/roboto_diff_drive_controller/odom", Odometry,self.odometry_callback)
        #self.img = self.debug_image

        kernel = np.ones((3,3),np.uint8)

        #self.img = cv2.dilate(self.img,kernel,iterations = 1)
        #self.img = cv2.erode(self.img,kernel,iterations = 3)

        #self.debug_image =self.img


    def debugging(self, rrt):


        if self.fig == None:
            plt.ion()
            plt.show()
            self.fig = plt.figure()
            self.ax = self.fig.add_axes([0,0,1,1])

        print("Drawing...")
        self.ax.imshow(self.debug_image, cmap=cm.Greys_r)
        plt.pause(0.001)
        plt.cla()



        self.robot_pose = np.array([10,10])
        self.robot_target = np.array([190,100])
        self.positionmap = [0,0]



        path = rrt.planning(animation=show_animation)
        if path is None:
            print("Cannot find path")
        else:
            print("found path!!")
            return path



    def callback(self, data):

        path=[]

        map = np.array(data.data)
        self.width = data.info.width
        self.height = data.info.height

        self.positionmap = [round(data.info.origin.position.x/0.05), round(data.info.origin.position.y/0.05)]

        resize = np.array(np.uint8(np.resize(map, [self.height,self.width])))
        self.img = 255 - resize

        # specify a threshold 0-255
        threshold = 200
        # make all pixels < threshold black

        print(type(self.img ))

        #cv2.imshow("",self.img )
        #cv2.waitKey(0)

        kernel = np.ones((3,3),np.uint8)
        self.img = cv2.erode(self.img,kernel,iterations = 2)
        #self.img = cv2.dilate(self.img,kernel,iterations = 1)


        self.img = 255 * (self.img  > threshold)

        print(self.positionmap)
        #robposex = +4.0 / 0.05
        #robposey = -1.0 / 0.05

        tarposex = +2.5 / 0.05
        tarposey = -1.9 / 0.05

        #self.robot_pose = np.array([robposex - self.positionmap[0], robposey - self.positionmap[1]])
        self.robot_target = np.array([tarposex - self.positionmap[0], tarposey - self.positionmap[1]])

        #print(self.robot_pose)
        print(self.robot_target)

        if self.fig == None:
            self.fig = plt.figure()
            self.fig.clf()
            self.ax = self.fig.add_subplot(1, 1, 1)
            self.ax.axis('image')
            self.ax.imshow(self.img, cmap=cm.Greys_r)
            print("INSIDE                     DFSDFSDGRGERHRTJSRTJRTSJRTJ")


        self.fig.clf()
        #plt.gca().invert_yaxis()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ax.imshow(self.img, cmap=cm.Greys_r)

        self.fig.canvas.draw()

        if self.Odometry:

            #TODO
            pass


        NoneType = type(None)

        if type(path) != NoneType:
            self.send(path)
        pass


    def odometry_callback(self, data):
        self.Odometry = True
        self.actual_pose = data
        pose = data.pose.pose.position
        self.robot_pose = np.array([pose.x /0.05 - self.positionmap[0], pose.y/0.05  - self.positionmap[1]])

    def send_image_path(self, path_image):

        image_message = self.bridge.cv2_to_imgmsg(path_image, "bgr8")
        try:
            self.path_image_pub.publish(image_message)
        except rospy.ROSInterruptException as ros_e :
            print(ros_e)

        self.rate.sleep()

    def send(self, path):
        """
        Publish the ROS message containing the waypoints
        """
        msg = Path()
        msg.header.frame_id = "path"
        msg.header.stamp = rospy.Time.now()
        for pos in path:
            pose = PoseStamped()
            pose.pose.position.x = pos[0]
            pose.pose.position.y = pos[1]
            pose.pose.position.z = 0

            msg.poses.append(pose)

        try:
            self.path_pub.publish(msg)
        except rospy.ROSInterruptException as ros_e :
            print(ros_e)
        self.rate.sleep()

def main():
    rospy.init_node('RRT_Node', anonymous=True)
    rrt = RRT_ROS()

    # ====Search Path with RRT====
    obstacle_list = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1),
        (6, 12, 1),
    ]  # [x,y,size(radius)]

    rrt_star = RRTStar(start=[0, 0],
                    goal=[6, 10],
                    rand_area=[-2, 15],
                    obstacle_list=obstacle_list,
                    expand_dis=1)

    print("Starting RRT Node ...")



    try:
        #rrt.send(np.array([[1,1],[2,1],[2,0],[1,0]]))
        while not rospy.is_shutdown():

            path = rrt.debugging(rrt_star)
            # Draw final path
            if show_animation:
                rrt_star.draw_graph()
                plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')
                plt.grid(True)
        plt.show()

        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
