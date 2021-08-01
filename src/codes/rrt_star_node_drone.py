#!/usr/bin/env python
import roslib
import tf2_ros
import rospy
import tf2_geometry_msgs
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib
import time
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
        self.rate = rospy.Rate(0.05)
        self.bridge = CvBridge()

        self.rrt_star = RrtStar((0,0), (0,0), 1500, 0.1, 1000, 3000)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.positionmap = [0,0]
        self.object_target = Node([7/0.05,-5/0.05])
        self.x_start = []
        self.x_goal = []
        self.real_start = []
        self.real_goal = []

        self.img2 = []

        self.path_pub = rospy.Publisher("/rrt/path_drone",Path, queue_size=1)
        self.path_image_pub = rospy.Publisher("/debug/image_path_drone",Image, queue_size=1)

        #self.image_sub = rospy.Subscriber("/rtabmap/grid_map",OccupancyGrid, self.callback, queue_size=1, buff_size=1)
        self.image_sub = rospy.Subscriber("/bebop/image_map",Image, self.callback_image, queue_size=1, buff_size=1)
        self.map_start_sub = rospy.Subscriber("/bebop/start_map", Point,self.callback_map_start, queue_size=1, buff_size=1)
        self.map_goal_sub= rospy.Subscriber('/bebop/goal_map', Point, self.callback_map_goal, queue_size=1, buff_size=1)
        self.start_sub = rospy.Subscriber("/bebop/start_map", Point,self.callback_start, queue_size=1, buff_size=1)
        self.goal_sub= rospy.Subscriber('/bebop/goal', Point, self.callback_goal, queue_size=1, buff_size=1)

    def getTargetRandomPose(self):

        delta = 5
        node = Node((750, 250))
        return node


    def callback(self, data):
        pass

    def callback_image(self, data):
        print("callback imagen")
        print(self.x_start != [], self.x_goal != [], self.real_goal != [])
        if self.x_start != [] and self.x_goal != [] and self.real_goal != []:
            print("AAAAAAAAAAAAAAAAAA")
            path=[]

            #self.example_function()
            map = np.array(data.data)
            self.width = data.width
            self.height = data.height
            size = [self.height,self.width ]
            self.fig = None
            self.positionmap = [-self.x_start.x,-self.x_start.y]
            self.z=0
            cv_image = self.bridge.imgmsg_to_cv2(data, 'mono8')
            self.img = np.array(np.uint8(np.resize(cv_image, [self.height,self.width ])))
            self.robot_pose = np.array([0,0])
            self.img = (255-self.img)

            d_x = self.real_goal.x / (self.x_goal.x-self.x_start.x)
            d_y = self.real_goal.y / (self.x_goal.y-self.x_start.y)

            print(d_x, d_y)
            #cv2.imshow('image',self.img)
            #cv2.waitKey(0)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))

            self.img = cv2.erode(self.img,kernel,iterations = 3)
            #self.img = cv2.dilate(self.img,kernel,iterations = 1)

            self.rrt_star.env.img = self.img
            self.rrt_star.utils.img = self.img
            self.rrt_star.plotting.img = self.img
            self.rrt_star.x_range = (0,size[1])
            self.rrt_star.y_range = (0,size[0])

            self.rrt_star.env.x_range = self.rrt_star.x_range
            self.rrt_star.env.y_range = self.rrt_star.y_range

            start_time = time.time()

            size = (np.shape(self.img))
            self.rrt_star.env.img = self.img
            self.rrt_star.utils.img = self.img
            self.rrt_star.plotting.img = self.img
            self.rrt_star.x_range = (0,size[1])
            self.rrt_star.y_range = (0,size[0])

            self.rrt_star.env.x_range = self.rrt_star.x_range
            self.rrt_star.env.y_range = self.rrt_star.y_range

            #self.x_start = Node(self.robot_pose)  # Starting node
            #self.x_goal = Node(self.robot_target)
            #x_start = (18, 20)  # Starting node
            #x_goal = (190, 125)
            self.rrt_star.selectStartGoalPoints(self.x_start , self.x_goal )
            path = self.rrt_star.planning()

            if path is None:
                print("Cannot find path")

            if path is not None:
                print("found path!!")

                odomcoor = np.array(path)
                odomcoorx = np.array([odomcoor[:,0] + self.positionmap[0]])*d_x
                odomcoory = np.array([odomcoor[:,1] + self.positionmap[1]])*d_y
                odomcoor = np.append(odomcoorx , odomcoory, axis= 0)
                path = odomcoor.T
                print 'Final path:', odomcoor.T

                self.send(path[::-1])


                img2 = np.fromstring(self.rrt_star.plotting.fig.canvas.tostring_rgb(), dtype=np.uint8,
                      sep='')

                ncols, nrows = self.rrt_star.plotting.fig.canvas.get_width_height()
                img2  = img2.reshape(self.rrt_star.plotting.fig.canvas.get_width_height()[::-1] + (3,))

                self.img2 = cv2.cvtColor(img2,cv2.COLOR_RGB2BGR)
                #rospy.signal_shutdown("End node")

                return path



        # NoneType = type(None)
        #
        # if type(path) != NoneType:
        #     self.send(path)
        # pass


        #self.example_function(data.pose.pose, 'map', 'odom')
    def callback_goal(self, data):
        self.real_goal = Node([data.x, data.y])
        #print("callback_goal")

    def callback_start(self, data):
        self.real_start = Node([data.x, data.y])
        #print("callback_start")

    def callback_map_goal(self, data):
        self.x_goal = Node([data.x, data.y])
        if self.img2 != []:
            self.send_image_path(self.img2)
        #print("callback_goal_map")

    def callback_map_start(self, data):
        self.x_start = Node([data.x, data.y])
        #print("callback_start_map")

    def send_image_path(self, path_image):

        image_message = self.bridge.cv2_to_imgmsg(path_image, "bgr8")
        try:
            self.path_image_pub.publish(image_message)
        except rospy.ROSInterruptException as ros_e :
            print(ros_e)

        #self.rate.sleep()

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
        #     rrt.send_image_path(rrt.img2)
        #

        #rrt.debugging()

        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
