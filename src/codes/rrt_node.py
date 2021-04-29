#!/usr/bin/env python
import roslib
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from matplotlib import cm
from scipy.misc import imread
import random, sys, math, os.path
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Point, Pose, PoseStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from PIL import Image as im

MAP_IMG = './maze.jpg' # Black and white image for a map
MIN_NUM_VERT = 10 # Minimum number of vertex in the graph
MAX_NUM_VERT = 1000 # Maximum number of vertex in the graph
STEP_DISTANCE = 10 # Maximum distance between two vertex
SEED = None # For random numbers

class RRT:

    def __init__(self):

        self.Odometry = False
        self.rate = rospy.Rate(1)
        #########
        self.debug_image = imread(MAP_IMG,mode="L")
        print(type(self.debug_image ))
        ########

        self.bridge = CvBridge()
        self.fig = None
        self.fig = plt.figure()
        self.fig.clf()
        self.ax = self.fig.add_subplot(1, 1, 1)

        self.image_sub = rospy.Subscriber("/rtabmap/grid_map",OccupancyGrid,self.callback)
        self.path_pub = rospy.Publisher("/rrt/path",Path, queue_size=1)
        self.path_image_pub = rospy.Publisher("/debug/image_path",Image, queue_size=1)
        self.odom_sub = rospy.Subscriber("/roboto_diff_drive_controller/odom", Odometry,self.odometry_callback)
        #self.img = self.debug_image

        kernel = np.ones((3,3),np.uint8)

        #self.img = cv2.dilate(self.img,kernel,iterations = 1)
        #self.img = cv2.erode(self.img,kernel,iterations = 3)

        #self.debug_image =self.img


    def debugging(self):

        self.fig.clf()
        self.ax = self.fig.add_subplot(1, 1, 1)
        #self.ax.axis('image')
        self.ax.imshow(self.debug_image, cmap=cm.Greys_r)
        self.fig.canvas.draw()

        self.robot_pose = np.array([10,10])
        self.robot_target = np.array([190,100])
        self.positionmap = [0,0]

        ##################################

        start, goal = self.selectStartGoalPoints(self.ax, self.img)
        path = self.rapidlyExploringRandomTree(self.ax, self.img, start, goal, seed=SEED)
        NoneType = type(None)
        if type(path) != NoneType:
            self.send(path)

    def callback(self, data):

        path = None
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

            start, goal = self.selectStartGoalPoints(self.ax, self.img)
            path = self.rapidlyExploringRandomTree(self.ax, self.img, start, goal, seed=SEED)

        NoneType = type(None)

        if type(path) != NoneType:
            self.send(path)
        pass

    def odometry_callback(self, data):
        self.Odometry = True
        self.actual_pose = data
        pose = data.pose.pose.position
        self.robot_pose = np.array([pose.x /0.05 - self.positionmap[0], pose.y/0.05  - self.positionmap[1]])
        #print("robot pose relative to map: ", self.robot_pose)



    def send_image_path(self, path_image):

        """
        Publish the ROS message containing the waypoints
        """

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

    def rapidlyExploringRandomTree(self, ax, img, start, goal, seed=None):
      hundreds = 1000
      random.seed(seed)
      points = []
      graph = []
      points.append(start)
      graph.append((start, []))
      print 'Generating and conecting random points'
      occupied = True
      phaseTwo = False

      # Phase two values (points 5 step distances around the goal point)
      minX = max(goal[0] - 5 * STEP_DISTANCE, 0)
      maxX = min(goal[0] + 5 * STEP_DISTANCE, len(img[0]) - 1)
      minY = max(goal[1] - 5 * STEP_DISTANCE, 0)
      maxY = min(goal[1] + 5 * STEP_DISTANCE, len(img) - 1)

      i = 0
      while (goal not in points) and (len(points) < MAX_NUM_VERT):
        if (i % 1000) == 0:
          print i, 'points randomly generated'

        if (len(points) % hundreds) == 0:
          print len(points), 'vertex generated'
          hundreds = hundreds + 1000

        while(occupied):
          if phaseTwo and (random.random() > 0.80):
            point = [ random.randint(minX, maxX), random.randint(minY, maxY) ]
          else:
            point = [ random.randint(0, self.width - 1), random.randint(0, self.height - 1) ]
          #print(np.shape(self.img))
          if(self.img[point[1]][point[0]] >= 200):
            occupied = False

        occupied = True

        nearest = self.findNearestPoint(points, point)
        newPoints = self.connectPoints(point, nearest, img)
        self.addToGraph(self.ax, graph, newPoints, point)
        newPoints.pop(0) # The first element is already in the points list
        points.extend(newPoints)

        i = i + 1

        if len(points) >= MIN_NUM_VERT:
          if not phaseTwo:
            print 'Phase Two'
          phaseTwo = True

        if phaseTwo:
          nearest = self.findNearestPoint(points, goal)
          newPoints = self.connectPoints(goal, nearest, img)
          self.addToGraph(self.ax, graph, newPoints, goal)
          newPoints.pop(0)
          points.extend(newPoints)
          #plt.draw()

      if goal in points:
        print 'Goal found, total vertex in graph:', len(points), 'total random points generated:', i
        #print(start)
        path = self.searchPath(graph, start, [start])
        NoneType = type(None)
        if (type(path) == NoneType):
            print("Fail to connect....................................)")
            return
        for i in range(len(path)-1):
          self.ax.plot([ path[i][0], path[i+1][0] ], [ path[i][1], path[i+1][1] ], color='b', linestyle='-', linewidth=2)
          pass
          #plt.draw()

        print 'Showing resulting map'
        print 'Final path:', path
        odomcoor = np.array(path)
        odomcoorx = np.array([odomcoor[:,0] + self.positionmap[0]])*0.05
        odomcoory = np.array([odomcoor[:,1] + self.positionmap[1]])*0.05
        odomcoor = np.append(odomcoorx , odomcoory, axis= 0)
        print 'Final path:', odomcoor.T
        print 'The final path is made from:', len(path),'connected points'
      else:
        path = None
        print 'Reached maximum number of vertex and goal was not found'
        print 'Total vertex in graph:', len(points), 'total random points generated:', i
        print 'Showing resulting map'

      self.fig.canvas.draw()
      img2 = np.fromstring(self.fig.canvas.tostring_rgb(), dtype=np.uint8,
            sep='')

      ncols, nrows = self.fig.canvas.get_width_height()
      img2  = img2.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))

      img2 = cv2.cvtColor(img2,cv2.COLOR_RGB2BGR)
      self.send_image_path(img2)

      return odomcoor.T

    def searchPath(self, graph, point, path):

      for i in graph:
        if point == i[0]:
          p = i

      if p[0] == graph[-1][0]:

        return path

      for link in p[1]:
        path.append(link)
        finalPath = self.searchPath(graph, link, path)

        if finalPath != None:
          return finalPath
        else:
          #print(graph)
          #print(point)
          #print(path)
          path.pop()


    def addToGraph(self, ax, graph, newPoints, point):
      if len(newPoints) > 1: # If there is anything to add to the graph
        for p in range(len(newPoints) - 1):
          nearest = [ nearest for nearest in graph if (nearest[0] == [ newPoints[p][0], newPoints[p][1] ]) ]
          nearest[0][1].append(newPoints[p + 1])
          graph.append((newPoints[p + 1], []))


          if not p==0:
            self.ax.plot(newPoints[p][0], newPoints[p][1], '+k') # First point is already painted
          self.ax.plot([ newPoints[p][0], newPoints[p+1][0] ], [ newPoints[p][1], newPoints[p+1][1] ], color='k', linestyle='-', linewidth=1)

        if point in newPoints:
          self.ax.plot(point[0], point[1], '.g') # Last point is green
        else:
          self.ax.plot(newPoints[p + 1][0], newPoints[p + 1][1], '+k') # Last point is not green


    def connectPoints(self, a, b, img):
      newPoints = []
      newPoints.append([ b[0], b[1] ])
      step = [ (a[0] - b[0]) / float(STEP_DISTANCE), (a[1] - b[1]) / float(STEP_DISTANCE) ]

      # Set small steps to check for walls
      pointsNeeded = int(math.floor(max(math.fabs(step[0]), math.fabs(step[1]))))

      if math.fabs(step[0]) > math.fabs(step[1]):
        if step[0] >= 0:
          step = [ 1, step[1] / math.fabs(step[0]) ]
        else:
          step = [ -1, step[1] / math.fabs(step[0]) ]

      else:
        if step[1] >= 0:
          step = [ step[0] / math.fabs(step[1]), 1 ]
        else:
          step = [ step[0]/math.fabs(step[1]), -1 ]

      blocked = False
      for i in range(pointsNeeded+1): # Creates points between graph and solitary point
        for j in range(STEP_DISTANCE): # Check if there are walls between points
          coordX = round(newPoints[i][0] + step[0] * j)
          coordY = round(newPoints[i][1] + step[1] * j)

          if coordX == a[0] and coordY == a[1]:
            break
          if coordY >= len(img) or coordX >= len(img[0]):
            break
          if img[int(coordY)][int(coordX)] < 255:
            blocked = True
          if blocked:
            break

        if blocked:
          break
        if not (coordX == a[0] and coordY == a[1]):
          newPoints.append([ newPoints[i][0]+(step[0]*STEP_DISTANCE), newPoints[i][1]+(step[1]*STEP_DISTANCE) ])

      if not blocked:
        newPoints.append([ a[0], a[1] ])
      return newPoints

    def findNearestPoint(self, points, point):
      best = (sys.maxint, sys.maxint, sys.maxint)
      for p in points:
        if p == point:
          continue
        dist = math.sqrt((p[0] - point[0]) ** 2 + (p[1] - point[1]) ** 2)
        if dist < best[2]:
          best = (p[0], p[1], dist)
      return (best[0], best[1])

    def selectStartGoalPoints(self, ax, img):
      print 'Select a starting point'
      self.ax.set_xlabel('Select a starting point')
      occupied = True
      while(occupied):
        point = self.robot_pose
        start = [ round(point[0]), round(point[1]) ]
        if(img[int(start[1])][int(start[0])] >= 200):
          occupied = False
          self.ax.plot(start[0], start[1], '.r')
        else:
          print 'Cannot place a starting point there'
          print(self.robot_pose)
          print(self.positionmap)
          print((img[int(start[1])][int(start[0])]))

          self.ax.set_xlabel('Cannot place a starting point there, choose another point')

      print 'Select a goal point'
      self.ax.set_xlabel('Select a goal point')
      occupied = True
      while(occupied):
        point = self.robot_target
        goal = [ round(point[0]), round(point[1]) ]
        if(img[int(goal[1])][int(goal[0])] >= 200):
          occupied = False
          self.ax.plot(goal[0], goal[1], '.b')
        else:
          print 'Cannot place a goal point there'
          self.ax.set_xlabel('Cannot place a goal point there, choose another point')

      #plt.draw()
      return start, goal

def main():
    rospy.init_node('RRT_Node', anonymous=True)
    rrt = RRT()
    print("Starting RRT Node ...")
    try:
        #rrt.send(np.array([[1,1],[2,1],[2,0],[1,0]]))
        # while not rospy.is_shutdown():
        #     rrt.debugging()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
