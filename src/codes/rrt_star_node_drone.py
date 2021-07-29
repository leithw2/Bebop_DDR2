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

        self.rrt_star = RrtStar((0,0), (0,0), 1500, 0.1, 1000, 4000)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.positionmap = [0,0]
        self.object_target = Node([7/0.05,-5/0.05])
        self.x_start = Node([0,0])
        self.x_goal = Node([0,0])

        self.path_pub = rospy.Publisher("/rrt/path",Path, queue_size=1)
        self.path_image_pub = rospy.Publisher("/debug/image_path",Image, queue_size=1)
        self.image_sub = rospy.Subscriber("/rtabmap/grid_map",OccupancyGrid, self.callback, queue_size=1, buff_size=1)
        self.image_drone_sub = rospy.Subscriber("/bebop/image_map",Image, self.callback_image_drone, queue_size=1, buff_size=1)
        self.odom_sub = rospy.Subscriber("/roboto_diff_drive_controller/odom", Odometry,self.odometry_callback)
        self.goal_pose_sub= rospy.Subscriber('/goal/pose', Point, self.goal_pose_callback)

    def getTargetRandomPose(self):

        delta = 5
        node = Node((750, 250))
        return node


    def callback(self, data):
        pass

    def callback_image_drone(self, data):


        if True:
            path=[]

            #self.example_function()
            map = np.array(data.data)
            self.width = data.width
            self.height = data.height
            size = [self.height,self.width ]
            self.fig = None
            self.positionmap = [0,0]
            self.z=0
            cv_image = self.bridge.imgmsg_to_cv2(data, 'mono8')
            self.img = np.array(np.uint8(np.resize(cv_image, [self.height,self.width ])))
            self.robot_pose = np.array([350,710])
            self.img = (255-self.img)
            print(self.img[10][10])

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

            #print(self.rrt_star.x_range)
            #print(self.rrt_star.y_range)

            start_time = time.time()

            tarpose = self.getTargetRandomPose()

            #print("Tiempo: ", time.time() - start_time)

            #print("target pose")
            #print(tarpose.x, tarpose.y)
            if tarpose != []:
                tarposex = tarpose.x
                tarposey = tarpose.y
            else:
                return
            #tarposex = 0
            #tarposey = 0

            #self.robot_pose = np.array([robposex - self.positionmap[0], robposey - self.positionmap[1]])
            self.robot_target = np.array([tarposex, tarposey])

            #print(self.robot_pose)
            print(self.robot_target)

            print(self.width)
            print(self.height)

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
                for i in range(50):

                    #self.neighborsToBlack(5,tarposey,tarposex)
                    #plt.imshow(self.img, cmap=cm.Greys_r)
                    tarpose = self.getTargetRandomPose()

                    if tarpose != []:
                        tarposex = tarpose.x
                        tarposey = tarpose.y
                    else:
                        continue

                    self.robot_target = np.array([tarposex, tarposey])
                    self.x_goal = Node(self.robot_target)
                    self.rrt_star.selectStartGoalPoints(self.x_start , self.x_goal )
                    path = self.rrt_star.planning()

                    if path is not None:
                        break

            if path is not None:
                print("found path!!")

                odomcoor = np.array(path)
                odomcoorx = np.array([odomcoor[:,0] + self.positionmap[0]])*0.05
                odomcoory = np.array([odomcoor[:,1] + self.positionmap[1]])*0.05
                odomcoor = np.append(odomcoorx , odomcoory, axis= 0)
                path = odomcoor.T
                print 'Final path:', odomcoor.T

                self.send(path[::-1])
                rospy.signal_shutdown("End node")

                img2 = np.fromstring(self.rrt_star.plotting.fig.canvas.tostring_rgb(), dtype=np.uint8,
                      sep='')

                ncols, nrows = self.rrt_star.plotting.fig.canvas.get_width_height()
                img2  = img2.reshape(self.rrt_star.plotting.fig.canvas.get_width_height()[::-1] + (3,))

                self.img2 = cv2.cvtColor(img2,cv2.COLOR_RGB2BGR)
                self.send_image_path(self.img2)

                return path

            pass


        # NoneType = type(None)
        #
        # if type(path) != NoneType:
        #     self.send(path)
        # pass


    def odometry_callback(self, data):
        pose = []
        self.actual_pose = data

        pose = self.example_function(data.pose.pose, 'map', 'odom')
        if pose is not None:
            self.Odometry = True
            pose = pose.pose.position
            self.robot_pose = np.array([pose.x /0.05 - self.positionmap[0], pose.y/0.05  - self.positionmap[1]])
        else:
            self.Odometry = False
        #print(pose.x,pose.y)

        #self.example_function(data.pose.pose, 'map', 'odom')
    def goal_pose_callback(self, data):
        self.object_target = Node([data.x/0.05, data.y/0.05])

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
        #     rrt.send_image_path(rrt.img2)
        #

        #rrt.debugging()

        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
