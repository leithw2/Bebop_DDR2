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

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "")

MAP_IMG = './maze.jpg' # Black and white image for a map

try:
    from rrt_star import RrtStar, Node
except ImportError:
    raise

show_animation = True

class RRT_ROS:

    def __init__(self):

        self.Odometry = False
        self.rate = rospy.Rate(1)
        #########
        self.debug_image = imread(MAP_IMG,mode="L")
        ########

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/rtabmap/grid_map",OccupancyGrid,self.callback)
        self.path_pub = rospy.Publisher("/rrt/path",Path, queue_size=1)
        self.path_image_pub = rospy.Publisher("/debug/image_path",Image, queue_size=1)
        self.odom_sub = rospy.Subscriber("/roboto_diff_drive_controller/odom", Odometry,self.odometry_callback)

    def debugging(self, rrt):


        self.robot_pose = np.array([10,10])
        self.robot_target = np.array([190,100])
        self.positionmap = [0,0]

        self.img = imread(MAP_IMG, mode="L")

        kernel = np.ones((3,3),np.uint8)
        self.img = cv2.dilate(self.img,kernel,iterations = 1)
        self.img = cv2.erode(self.img,kernel,iterations = 1)

        size = (np.shape(self.img))

        rrt.env.img = self.img
        rrt.utils.img = self.img
        rrt.plotting.img = self.img
        rrt.x_range = (0,size[0])
        rrt.y_range = (0,size[1])

        rrt.env.x_range = rrt.x_range
        rrt.env.y_range = rrt.y_range

        x_start = (18, 8)  # Starting node
        x_goal = (190, 125)

        rrt.selectStartGoalPoints(Node(x_start), Node(x_goal) )

        path = rrt.planning()
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

        self.img = np.array(np.uint8(np.resize(map, [self.height,self.width])))

        kernel = np.ones((3,3),np.uint8)
        #self.img = cv2.erode(self.img,kernel,iterations = 2)
        self.img = cv2.dilate(self.img,kernel,iterations = 2)
        self.img = 255 - self.img

        # specify a threshold 0-255
        threshold = 200
        # make all pixels < threshold black

        self.img = 255 * (self.img  > threshold)

        tarposex = +2.5 / 0.05
        tarposey = -1.9 / 0.05

        #self.robot_pose = np.array([robposex - self.positionmap[0], robposey - self.positionmap[1]])
        self.robot_target = np.array([tarposex - self.positionmap[0], tarposey - self.positionmap[1]])

        #print(self.robot_pose)
        print(self.robot_target)

        if self.Odometry:

            size = (np.shape(self.img))

            rrt.env.img = self.img
            rrt.utils.img = self.img
            rrt.plotting.img = self.img
            rrt.x_range = (0,size[0])
            rrt.y_range = (0,size[1])

            rrt.env.x_range = rrt.x_range
            rrt.env.y_range = rrt.y_range


            path = rrt.planning()
            if path is None:
                print("Cannot find path")
            else:
                print("found path!!")
                return path

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
    x_start = (18, 8)  # Starting node
    x_goal = (125, 125)  # Goal node
    rrt_star = RrtStar(x_start, x_goal, 100, 0.02, 100, 4000)

    print("Starting RRT Star Node ...")

    try:
        while not rospy.is_shutdown():

            rrt.debugging(rrt_star)

        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


        v2.destroyAllWindows()

if __name__ == '__main__':
    main()
