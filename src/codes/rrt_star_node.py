#!/usr/bin/env python
import roslib
import tf2_ros
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

MAP_IMG = './test_map4.png' # Black and white image for a map

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
        self.rrt_star = RrtStar((0,0), (0,0), 10, 0.02, 100, 4000)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.positionmap = [0,0]
        self.object_target =Node([190,190])
        self.x_start = Node([0,0])
        self.x_goal = Node([0,0])



    def example_function(self):

        try:
            print("trying..")
            trans = self.tfBuffer.lookup_transform('map', 'odom', rospy.Time())
            print("trying.. end")
            print(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass



    def debugging(self):


        self.robot_pose = np.array([10,10])
        self.robot_target = np.array([190,100])
        self.positionmap = [0,0]

        self.img = imread(MAP_IMG, mode="L")

        kernel = np.ones((2,2),np.uint8)
        self.img = cv2.dilate(self.img,kernel,iterations = 1)
        self.img = cv2.erode(self.img,kernel,iterations = 2)

        size = (np.shape(self.img))
        self.rrt_star.env.img = self.img
        self.rrt_star.utils.img = self.img
        self.rrt_star.plotting.img = self.img
        self.rrt_star.x_range = (0,size[1])
        self.rrt_star.y_range = (0,size[0])

        self.rrt_star.env.x_range = self.rrt_star.x_range
        self.rrt_star.env.y_range = self.rrt_star.y_range

        robposex = 35.0
        robposey = 28.0

        tarpose = self.getTargetRandomPose()
        print("target pose")
        print(tarpose.x, tarpose.y)
        tarposex = tarpose.x
        tarposey = tarpose.y

        self.robot_pose = np.array([robposex - self.positionmap[0], robposey - self.positionmap[1]])
        self.robot_target = np.array([tarposex - self.positionmap[0], tarposey - self.positionmap[1]])


        self.x_start = Node(self.robot_pose)  # Starting node
        self.x_goal = Node(self.robot_target)

        self.rrt_star.selectStartGoalPoints(self.x_start, self.x_goal )
        path = self.rrt_star.planning()

        #self.rrt.selectStartGoalPoints(Node(x_start), Node(x_goal) )

        if path is None:
            print("Cannot find path")
        else:
            print("found path!!")
            return path

    def getTargetRandomPose(self):
        delta = 5
        min_dist = sys.maxint
        node_dist_min = []

        for k in range(5000):
            node = Node((np.random.randint(self.rrt_star.x_range[0] + delta, self.rrt_star.x_range[1] - delta),
                         np.random.randint(self.rrt_star.y_range[0] + delta, self.rrt_star.y_range[1] - delta)))

            if(self.img[int(node.y)][int(node.x)] >= 200):
                my_list1 = np.array(self.neighbors(5, node.y, node.x))
                print(my_list1)


                if((my_list1 <= 100).any()): #if black
                    print("continue")
                    continue

                if((my_list1 >= 100).any() and (my_list1 <= 200).any()):
                    robotToNode = self.rrt_star.utils.get_dist(self.x_start,node)
                    nodeToObject = self.rrt_star.utils.get_dist(node, self.object_target)
                    print("Grey")
                    if (robotToNode + nodeToObject < min_dist):
                        min_dist = robotToNode + nodeToObject #if gray
                        print(my_list1)
                        print(node.x, node.y)
                        print(min_dist)
                        node_dist_min = node


        if node_dist_min != []:
            return node_dist_min
        else:
            print("fail no point avalible")


    def neighbors(self, radius, rowNumber, columnNumber):
        a = self.img
        return [[a[i][j] if  i >= 0 and i < len(a) and j >= 0 and j < len(a[0]) else 0
            for j in range(columnNumber-1-radius, columnNumber+radius)]
                for i in range(rowNumber-1-radius, rowNumber+radius)]

    def callback(self, data):

        path=[]

        self.example_function()
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

        #robposex = 2.0 / 0.05
        #robposey = 0.0 / 0.05
        tarposex = +3.9 / 0.05
        tarposey = -1.0 / 0.05

        #self.robot_pose = np.array([robposex - self.positionmap[0], robposey - self.positionmap[1]])
        self.robot_target = np.array([tarposex - self.positionmap[0], tarposey - self.positionmap[1]])

        #print(self.robot_pose)
        print(self.robot_target)

        print(self.width)
        print(self.height)

        if self.Odometry:

            size = (np.shape(self.img))
            self.rrt_star.env.img = self.img
            self.rrt_star.utils.img = self.img
            self.rrt_star.plotting.img = self.img
            self.rrt_star.x_range = (0,size[1])
            self.rrt_star.y_range = (0,size[0])

            self.rrt_star.env.x_range = self.rrt_star.x_range
            self.rrt_star.env.y_range = self.rrt_star.y_range

            self.x_start = Node(self.robot_pose)  # Starting node
            self.x_goal = Node(self.robot_target)
            #x_start = (18, 20)  # Starting node
            #x_goal = (190, 125)
            self.rrt_star.selectStartGoalPoints(self.x_start , self.x_goal )
            path = self.rrt_star.planning()
            if path is None:
                print("Cannot find path")
            else:
                print("found path!!")

                odomcoor = np.array(path)
                odomcoorx = np.array([odomcoor[:,0] + self.positionmap[0]])*0.05
                odomcoory = np.array([odomcoor[:,1] + self.positionmap[1]])*0.05
                odomcoor = np.append(odomcoorx , odomcoory, axis= 0)
                path = odomcoor.T
                print 'Final path:', odomcoor.T

                self.send(path[::-1])
                return path

            pass


        # NoneType = type(None)
        #
        # if type(path) != NoneType:
        #     self.send(path)
        # pass


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


    print("Starting RRT Star Node ...")

    try:
        # while not rospy.is_shutdown():
        #
        #

        rrt.debugging()

        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


        v2.destroyAllWindows()

if __name__ == '__main__':
    main()
