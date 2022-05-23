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
from geometry_msgs.msg import PoseArray, Point, Pose, PoseStamped, PoseWithCovarianceStamped
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

        self.rrt_star = RrtStar((0,0), (0,0), 150, 0.2, 20, 2000)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.positionmap = [0,0]
        self.object_target = Node([0,0])
        self.robot_pose = Node([0,0])
        self.x_start = Node([0,0])
        self.x_goal = Node([0,0])
        self.wayPoints = []
        self.haveWayPoints = False
        self.haveMap = False


        self.path_pub = rospy.Publisher("/rrt/path",Path, queue_size=1)
        self.path_image_pub = rospy.Publisher("/debug/ddr_path",Image, queue_size=1)

        self.image_sub = rospy.Subscriber("/rtabmap/grid_map",OccupancyGrid, self.callback, queue_size=1, buff_size=1)
        #self.image_drone_sub = rospy.Subscriber("/bebop/image_map",Image, self.callback_image_drone, queue_size=1, buff_size=1)

        self.poseWhitCovariance_sub = rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped,self.poseWhitCovariance_callback)
        self.odom_sub = rospy.Subscriber("/roboto_diff_drive_controller/odom", Odometry,self.odometry_callback)
        self.goal_pose_sub= rospy.Subscriber('/goal/pose', Point, self.goal_pose_callback)
        self.path_sub = rospy.Subscriber("/rrt/path_drone", Path ,self.callback_drone_path)



    def example_function(self, pose, to, from_):

        try:
            trans = self.tfBuffer.lookup_transform(to, from_, rospy.Time())
            ps = PoseStamped()
            ps.pose = pose
            ps.header.frame_id = 'odom'
            return tf2_geometry_msgs.do_transform_pose(ps, trans)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass


    def getWaypointRandomPose(self):

        delta = 10
        min_dist = sys.maxint
        node_dist_min = []
        point = self.object_target
        zonex = int(point.y)
        zoney = int(point.x)
        zonex = self.width
        zoney = self.height

        for k in range(1000):
            # node_x = np.random.randint(zonex - delta, zonex + delta)
            # if node_x >= self.width:
            #     node_x = self.width-1
            # if node_x < 0:
            #     node_x=0
            #
            # node_y = np.random.randint(zoney - delta, zoney + delta)
            # if node_y >= self.height:
            #     node_y = self.height-1
            # if node_y < 0:
            #     node_y=0
            node_x = np.random.randint(0, zonex)
            node_y = np.random.randint(0, zoney)
            node = Node((node_x,node_y ))

            if(self.img[int(node.y)][int(node.x)] >= 200):
                my_list1 = np.array(self.neighbors(2, node.y, node.x))
                #print(my_list1)

                if((my_list1 <= 100).any()): #if black
                    #print("continue")
                    continue

                if((my_list1 > 100).any() and (my_list1 <= 200).any()): #if gray

                    robotToNode = self.rrt_star.utils.get_dist(self.x_start,node)
                    nodeToObject = self.rrt_star.utils.get_dist(node, self.object_target)

                    if robotToNode < delta * 2 or robotToNode >= delta * 500 :
                        #print("node ", node.y, node.x)
                        #print("to short", robotToNode)
                        continue

                    if (robotToNode + nodeToObject < min_dist):
                        min_dist = robotToNode + nodeToObject #if gray
                        print("shortest",min_dist)
                        node_dist_min = node



        if node_dist_min != []:
            print("shortest_ node ",node_dist_min.y, node_dist_min.x)
            #print("shortest",self.rrt_star.utils.get_dist(self.x_start,node_dist_min))
            return node_dist_min
        else:
            print("fail no wayPoint avalible")
            return node_dist_min


    def neighbors(self, radius, rowNumber, columnNumber):
        a = self.img
        return [[a[i][j] if  i >= 0 and i < len(a) and j >= 0 and j < len(a[0]) else 0
            for j in range(columnNumber-1-radius, columnNumber+radius)]
                for i in range(rowNumber-1-radius,rowNumber+radius)]

    def neighborsToBlack(self, radius, rowNumber, columnNumber):
        for j in range(columnNumber-1-radius, columnNumber+radius):
            for i in range(rowNumber-1-radius, rowNumber+radius):
                self.img[i][j] = 0

    def neighborsToWhite(self, radius, rowNumber, columnNumber):
        for j in range(columnNumber-1-radius, columnNumber+radius):
            for i in range(rowNumber-1-radius, rowNumber+radius):
                self.img[i][j] = 255


    def callback(self, data):
        if not self.haveMap:
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
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))

            self.img = cv2.erode(self.img,kernel,iterations = 1)
            #self.img = cv2.dilate(self.img,kernel,iterations = 1)
            self.rrt_star.env.img = self.img
            self.rrt_star.utils.img = self.img
            self.rrt_star.plotting.img = self.img
            self.rrt_star.x_range = (0,size[1])
            self.rrt_star.y_range = (0,size[0])

            self.pathPlanning(data)

    def callback_drone_path(self, data):

        if not self.haveWayPoints:
            self.wayPoints = []
            for pose in data.poses:
                point = Node([(pose.pose.position.x - self.positionmap[0] )/self.respix,
                 -(pose.pose.position.y - self.positionmap[1])/self.respix])
                self.wayPoints.append(point)

            #print (self.wayPoints)

            print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
            #print(data.poses)

            self.haveWayPoints = True

    def pathPlanning(self, data):

        print("mapa recibido")

        if self.Odometry and self.haveWayPoints:
            self.haveMap = False
            path=[]
            print("waypoints recibiodos")

            self.rrt_star.env.x_range = self.rrt_star.x_range
            self.rrt_star.env.y_range = self.rrt_star.y_range

            wayPoint_x = self.wayPoints[-1].x - self.positionmap[0]
            wayPoint_y = self.wayPoints[-1].y - self.positionmap[1]

            if wayPoint_x >= self.width:
                wayPoint_x = self.width-1
            if wayPoint_x < 0:
                wayPoint_x = 0

            if wayPoint_y >= self.height:
                wayPoint_y = self.height-1
            if wayPoint_y < 0:
                wayPoint_y = 0

            self.object_target.x = wayPoint_x
            self.object_target.y = wayPoint_y

            start_time = time.time()
            print(self.width, self.height)
            print("waypoint targert")
            print(self.wayPoints[-1].x, self.wayPoints[-1].y )
            print(self.object_target.x, self.object_target.y )
            #tarpose = self.getTargetRandomPose()
            self.robot_start = np.array([self.robot_pose[1] - self.positionmap[0], self.robot_pose[0] - self.positionmap[1]])
            self.x_start = Node([self.robot_start[0], self.robot_start[1]])  # Starting node

            tarpose = self.getWaypointRandomPose()
            #print("Tiempo: ", time.time() - start_time)

            if tarpose != []:
                tarposex = tarpose.x
                tarposey = tarpose.y
            else:
                return

            self.robot_start = np.array([self.robot_pose[1] - self.positionmap[0], self.robot_pose[0] - self.positionmap[1]])
            self.robot_target = np.array([tarposex, tarposey])
            #self.robot_target = np.array([78 , 70])

            print("desde:" , self.robot_start)
            print("hasta:" ,self.robot_target)

            print("desde:" , self.robot_start*self.respix)
            print("hasta:" ,self.robot_target*self.respix)



            size = (np.shape(self.img))
            self.rrt_star.env.img = self.img
            self.rrt_star.utils.img = self.img
            self.rrt_star.plotting.img = self.img
            self.rrt_star.x_range = (0,size[1])
            self.rrt_star.y_range = (0,size[0])

            self.rrt_star.env.x_range = self.rrt_star.x_range
            self.rrt_star.env.y_range = self.rrt_star.y_range

            self.x_goal = Node(self.robot_target)
            #x_start = (18, 20)  # Starting node
            #x_goal = (190, 125)

            print(self.robot_start[0], self.robot_start[1])
            print(self.x_start.x, self.x_start.y)

            self.neighborsToWhite(4, int(self.x_start.y), int(self.x_start.x))
            #self.neighborsToBlack(2, int(self.object_target.x), int(self.object_target.y))
            #self.img[int(self.object_target.y)][int(self.object_target.x)] = 180
            self.img2 = cv2.cvtColor(self.img,cv2.COLOR_RGB2BGR)




            self.rrt_star.selectStartGoalPoints(self.x_start , self.x_goal )
            path = self.rrt_star.planning()

            if path is None:
                print("Cannot find path")
                for i in range(50):

                    self.neighborsToBlack(5,tarposey,tarposex)

                    tarpose = self.getWaypointRandomPose()

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

                # # Blue color in BGR
                # color = (255, 0, 0)
                #
                # # Line thickness of 2 px
                # thickness = 2
                #
                # cv2.polylines(self.img2,path , False, color, thickness)
                # plt.imshow(self.img2, cmap=cm.Greys_r)
                # plt.pause(1)
                self.send_image_path(self.img2)

                return path

            pass

        else:
            print("Esperando waypoints")
        # NoneType = type(None)
        #
        # if type(path) != NoneType:
        #     self.send(path)
        # pass


    def odometry_callback(self, data):
        # pose = []
        # self.actual_pose = data
        #
        # pose = self.example_function(data.pose.pose, 'map', 'odom')
        #
        # if pose is not None:
        #     self.Odometry = True
        #     pose = pose.pose.position
        #     self.robot_pose = np.array([pose.y /self.respix, pose.x/self.respix])
        #
        # else:
        #     self.Odometry = False
        pass

    def poseWhitCovariance_callback(self, data):
        pose = []
        self.actual_pose = data

        pose = data.pose.pose

        if pose is not None:
            self.Odometry = True
            pose = pose.position
            self.robot_pose = np.array([pose.y /self.respix, pose.x/self.respix])

        else:
            self.Odometry = False

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
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        i=0
        for pos in path:
            pose = PoseStamped()
            pose.header.seq = i
            pose.header.frame_id = "map"
            pose.header.stamp= rospy.Time.now()
            pose.pose.position.x = pos[0]
            pose.pose.position.y = pos[1]
            pose.pose.position.z = 0

            msg.poses.append(pose)
            i+=1

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
