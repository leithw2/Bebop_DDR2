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
        #########
        #self.debug_image = imread(MAP_IMG,mode="L")
        #self.img2 = cv2.cvtColor(self.debug_image,cv2.COLOR_RGB2BGR)
        ########
        self.respix = 0.05
        self.bridge = CvBridge()

        self.rrt_star = RrtStar((0,0), (0,0), 1500, 0.2, 200, 1000)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.positionmap = [0,0]
        self.object_target = Node([0,0])
        self.x_start = Node([0,0])
        self.x_goal = Node([0,0])
        self.wayPoints = []
        self.haveWayPoints = False


        self.path_pub = rospy.Publisher("/rrt/path",Path, queue_size=1)
        self.path_image_pub = rospy.Publisher("/debug/ddr_path",Image, queue_size=1)

        self.image_sub = rospy.Subscriber("/rtabmap/grid_map",OccupancyGrid, self.callback, queue_size=1, buff_size=1)
        #self.image_drone_sub = rospy.Subscriber("/bebop/image_map",Image, self.callback_image_drone, queue_size=1, buff_size=1)
        self.odom_sub = rospy.Subscriber("/roboto_diff_drive_controller/odom", Odometry,self.odometry_callback)
        self.goal_pose_sub= rospy.Subscriber('/goal/pose', Point, self.goal_pose_callback)
        self.path_sub = rospy.Subscriber("/rrt/path_drone", Path ,self.callback_drone_path)



    def example_function(self, pose, to, from_):

        try:
            #print("trying..")
            trans = self.tfBuffer.lookup_transform(to, from_, rospy.Time())
            #print("trying.. end")
            #print(trans)
            ps = PoseStamped()
            ps.pose = pose
            ps.header.frame_id = 'odom'
            return tf2_geometry_msgs.do_transform_pose(ps, trans)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass



    # def debugging(self):
    #
    #
    #     self.robot_pose = np.array([10,10])
    #     self.robot_target = np.array([100,190])
    #     self.positionmap = [0,0]
    #
    #     self.img = imread(MAP_IMG, mode="L")
    #
    #     kernel = np.ones((4,4),np.uint8)
    #     self.img = cv2.erode(self.img,kernel,iterations = 2)
    #     self.img = cv2.dilate(self.img,kernel,iterations = 2)
    #
    #
    #     size = (np.shape(self.img))
    #     self.rrt_star.env.img = self.img
    #     self.rrt_star.utils.img = self.img
    #     self.rrt_star.plotting.img = self.img
    #     self.rrt_star.x_range = (0,size[1])
    #     self.rrt_star.y_range = (0,size[0])
    #
    #     self.rrt_star.env.x_range = self.rrt_star.x_range
    #     self.rrt_star.env.y_range = self.rrt_star.y_range
    #
    #
    #
    #     robposex = 35.0
    #     robposey = 28.0
    #     start_time = time.time()
    #     tarpose = self.getTargetRandomPose()
    #     print("Tiempo: ", time.time() - start_time)
    #
    #     print("target pose")
    #     print(tarpose.x, tarpose.y)
    #     tarposex = tarpose.x
    #     tarposey = tarpose.y
    #
    #     self.robot_pose = np.array([robposex - self.positionmap[0], robposey - self.positionmap[1]])
    #     self.robot_target = np.array([tarposex - self.positionmap[0], tarposey - self.positionmap[1]])
    #
    #
    #     self.x_start = Node(self.robot_pose)  # Starting node
    #     self.x_goal = Node(self.robot_target)
    #
    #     self.rrt_star.selectStartGoalPoints(self.x_start, self.x_goal )
    #
    #     path = self.rrt_star.planning()
    #
    #     #self.rrt.selectStartGoalPoints(Node(x_start), Node(x_goal) )
    #
    #     if path is None:
    #         print("Cannot find path")
    #         while True:
    #             start_time = time.time()
    #
    #             tarpose = self.getTargetRandomPose()
    #             print("Tiempo: ", time.time() - start_time)
    #             pass
    #
    #     else:
    #         print("found path!!")
    #         return path

    def getTargetRandomPose(self):

        delta = 5
        min_dist = sys.maxint
        node_dist_min = []
        for k in range(10000):
            node = Node((np.random.randint(self.rrt_star.x_range[0] + delta, self.rrt_star.x_range[1] - delta),
                         np.random.randint(self.rrt_star.y_range[0] + delta, self.rrt_star.y_range[1] - delta)))

            if(self.img[int(node.y)][int(node.x)] >= 200):
                my_list1 = np.array(self.neighbors(2, node.y, node.x))
                #print(my_list1)

                if((my_list1 <= 100).any()): #if black
                    #print("continue")
                    continue

                if((my_list1 > 100).any() and (my_list1 <= 200).any()):

                    if ((node.x >= (self.object_target.x - delta)) and (node.x < (self.object_target.x + delta))):
                        if ((node.y >= (self.object_target.y - delta)) and (node.y < (self.object_target.y + delta))):
                            return self.object_target

                    robotToNode = self.rrt_star.utils.get_dist(self.x_start,node)
                    nodeToObject = self.rrt_star.utils.get_dist(node, self.object_target)

                    if robotToNode < delta * 1 or robotToNode >= delta * 50 :
                        print("node ", node.y, node.x)
                        print("to short", robotToNode)
                        continue

                    if (robotToNode + nodeToObject < min_dist):
                        min_dist = robotToNode + nodeToObject #if gray
                        #print(my_list1)
                        #print(node.x, node.y)
                        #print("shortest",min_dist)
                        node_dist_min = node



        if node_dist_min != []:
            #print("shortest_ node ",node_dist_min.y, node_dist_min.x)
            #print("shortest",self.rrt_star.utils.get_dist(self.x_start,node_dist_min))
            return node_dist_min
        else:
            print("fail no point avalible")
            return node_dist_min

    def getWaypointRandomPose(self):

        delta = 5
        min_dist = sys.maxint
        node_dist_min = []
        point = self.wayPoints[-3]
        print("Goal position")
        print(point)
        zonex = int(point[1]/self.respix - self.positionmap[0])
        zoney = int(point[0]/self.respix - self.positionmap[1])
        print("Goal position in new map")

        print(zonex)
        print(zoney)

        for k in range(100):
            node = Node((np.random.randint(zonex + delta, zonex - delta),
                         np.random.randint(zoney + delta, zoney - delta)))

            if(self.img[int(node.y)][int(node.x)] >= 200):
                my_list1 = np.array(self.neighbors(2, node.y, node.x))
                #print(my_list1)

                if((my_list1 <= 100).any()): #if black
                    #print("continue")
                    continue

                if((my_list1 > 100).any() and (my_list1 <= 200).any()):

                    # robotToNode = self.rrt_star.utils.get_dist(self.x_start,node)
                    # nodeToObject = self.rrt_star.utils.get_dist(node, self.object_target)
                    #
                    # if robotToNode < delta * 15 or robotToNode >= delta * 500 :
                    #     print("node ", node.y, node.x)
                    #     print("to short", robotToNode)
                    #     continue
                    #
                    # if (robotToNode + nodeToObject < min_dist):
                    #     min_dist = robotToNode + nodeToObject #if gray
                    #     #print(my_list1)
                    #     #print(node.x, node.y)
                    #     #print("shortest",min_dist)
                    node_dist_min = node



        if node_dist_min != []:
            #print("shortest_ node ",node_dist_min.y, node_dist_min.x)
            #print("shortest",self.rrt_star.utils.get_dist(self.x_start,node_dist_min))
            return node_dist_min
        else:
            print("fail no wayPoint avalible")
            return node_dist_min


    def neighbors(self, radius, rowNumber, columnNumber):
        a = self.img
        return [[a[i][j] if  i >= 0 and i < len(a) and j >= 0 and j < len(a[0]) else 0
            for j in range(columnNumber-1-radius, columnNumber+radius)]
                for i in range(rowNumber-1-radius, rowNumber+radius)]

    def neighborsToBlack(self, radius, rowNumber, columnNumber):
        for j in range(columnNumber-1-radius, columnNumber+radius):
            for i in range(rowNumber-1-radius, rowNumber+radius):
                self.img[i][j] = 0


    def callback(self, data):
        self.pathPlanning(data)

    def callback_drone_path(self, data):
        self.wayPoints = []
        for pose in data.poses:
            point = Node([pose.pose.position.x/self.respix, pose.pose.position.y/self.respix])
            self.wayPoints.append(point)

        #print (self.positions)
        #self.object_target = self.wayPoints[1]
        print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
        print(self.object_target.x, self.object_target.y )
        self.haveWayPoints= True

    def pathPlanning(self, data):

        print("Esperando waypoints")

        if self.Odometry: #and self.haveWayPoints:
            path=[]
            print("waypoints recibiodos")

            #self.example_function()
            map = np.array(data.data)
            self.width = data.info.width
            self.height = data.info.height
            size = [self.height,self.width ]

            self.positionmap = [round(data.info.origin.position.x/self.respix), round(data.info.origin.position.y/self.respix)]
            self.fig = None
            self.z=0
            self.img = np.array(np.uint8(np.resize(map, [self.height,self.width])))



            self.img = np.where(self.img == 255, 102, self.img)
            self.img = np.where(self.img == 0, 255, self.img)

            self.img = np.where(self.img == 100, 0, self.img)

            #print(self.img)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))

            self.img = cv2.erode(self.img,kernel,iterations = 1)
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
            self.object_target.x = self.object_target.x - self.positionmap[0]
            self.object_target.y = self.object_target.y - self.positionmap[1]
            tarpose = self.getTargetRandomPose()
            #tarpose = self.getWaypointRandomPose()
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
            self.robot_target = np.array([78 , 70])

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

                    self.neighborsToBlack(5,tarposey,tarposex)
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
                odomcoorx = np.array([odomcoor[:,0] + self.positionmap[0]])*self.respix
                odomcoory = np.array([odomcoor[:,1] + self.positionmap[1]])*self.respix
                odomcoor = np.append(odomcoorx , odomcoory, axis= 0)
                path = odomcoor.T
                print 'Final path:', odomcoor.T

                self.send(path[::-1])


                # img2 = np.fromstring(self.rrt_star.plotting.fig.canvas.tostring_rgb(), dtype=np.uint8,
                #       sep='')
                #
                # ncols, nrows = self.rrt_star.plotting.fig.canvas.get_width_height()
                # img2  = img2.reshape(self.rrt_star.plotting.fig.canvas.get_width_height()[::-1] + (3,))
                #
                # self.img2 = cv2.cvtColor(img2,cv2.COLOR_RGB2BGR)
                # self.send_image_path(self.img2)

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
            self.robot_pose = np.array([pose.x /self.respix - self.positionmap[0], pose.y/self.respix  - self.positionmap[1]])
            #print(pose)
            #print(self.robot_pose)
        else:
            self.Odometry = False
        #print(pose.x,pose.y)

        #self.example_function(data.pose.pose, 'map', 'odom')
    def goal_pose_callback(self, data):
        #self.object_target = Node([data.x/self.respix - self.positionmap[0], data.y/self.respix - self.positionmap[1]])
        pass

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
