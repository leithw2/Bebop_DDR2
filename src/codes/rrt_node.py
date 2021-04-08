#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from matplotlib import pyplot as ppl
from matplotlib import cm
from scipy.misc import imread
import random, sys, math, os.path
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as im

MAP_IMG = './gfg_dummy_pic.png' # Black and white image for a map
MIN_NUM_VERT = 20 # Minimum number of vertex in the graph
MAX_NUM_VERT = 1500 # Maximum number of vertex in the graph
STEP_DISTANCE = 10 # Maximum distance between two vertex
SEED = None # For random numbers

class RRT:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/rtabmap/grid_map",OccupancyGrid,self.callback)
        self.bridge = CvBridge()
        self.fig = ppl.gcf()
        self.fig.clf()
        self.ax = self.fig.add_subplot(1, 1, 1)
        #self.ax.imshow(self.img, cmap=cm.Greys_r)
        self.ax.axis('image')
        #print 'Map is', len(img[0]), 'x', len(img)

    def callback(self, data):
        #print(data)
        map = np.array(data.data)
        width = data.info.width
        height = data.info.height
        resize = np.uint8(np.resize(map, [height,width]))
        self.img = 255 - resize
        kernel = np.ones((10,10),np.uint8)
        self.img = cv2.dilate(self.img,kernel,iterations = 1)
        self.img = cv2.erode(self.img,kernel,iterations = 1)
        self.positionmap = [round(data.info.origin.position.x/0.05), round(data.info.origin.position.y/0.05)]
        print(self.positionmap)

        self.robot_pose = np.array([24 - self.positionmap[0],4 - self.positionmap[1]])
        self.robot_target = np.array([34 - self.positionmap[0],8 - self.positionmap[1]])
        #cv2.imshow("",255 - data)
        #resize = 255 - resize

        #img= resize
        #img.save('gfg_dummy_pic.png')
        #cv2.imshow("",grayimg)
        #cv2.imshow("",self.img)
        #cv2.waitKey(0)
        #print(np.array(data.data).shape)
        print(resize)
        start, goal = self.selectStartGoalPoints(self.ax, self.img)
        path = self.rapidlyExploringRandomTree(self.ax, self.img, start, goal, seed=SEED)

        pass

    def draw(self):
        self.fig.canvas.draw()
        self.fig.show()

    def rapidlyExploringRandomTree(self, ax, img, start, goal, seed=None):
      hundreds = 100
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
        if (i % 100) == 0:
          print i, 'points randomly generated'

        if (len(points) % hundreds) == 0:
          print len(points), 'vertex generated'
          hundreds = hundreds + 100

        while(occupied):
          if phaseTwo and (random.random() > 0.8):
            point = [ random.randint(minX, maxX), random.randint(minY, maxY) ]
          else:
            point = [ random.randint(0, len(img[0]) - 1), random.randint(0, len(img) - 1) ]

          if(img[point[1]][point[0]] == 255):
            occupied = False

        occupied = True

        nearest = self.findNearestPoint(points, point)
        newPoints = self.connectPoints(point, nearest, img)
        self.addToGraph(self.ax, graph, newPoints, point)
        newPoints.pop(0) # The first element is already in the points list
        points.extend(newPoints)
        #ppl.draw()
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
          #ppl.draw()

      if goal in points:
        print 'Goal found, total vertex in graph:', len(points), 'total random points generated:', i
        path = self.searchPath(graph, start, [start])

        for i in range(len(path)-1):
          self.ax.plot([ path[i][0], path[i+1][0] ], [ path[i][1], path[i+1][1] ], color='b', linestyle='-', linewidth=2)
          #ppl.draw()

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

      #ppl.show()
      return path


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
        if(img[int(start[1])][int(start[0])] == 255):
          occupied = False
          self.ax.plot(start[0], start[1], '.r')
        else:
          print 'Cannot place a starting point there'
          print(self.robot_pose)
          self.ax.set_xlabel('Cannot place a starting point there, choose another point')

      print 'Select a goal point'
      self.ax.set_xlabel('Select a goal point')
      occupied = True
      while(occupied):
        point = self.robot_target
        goal = [ round(point[0]), round(point[1]) ]
        if(img[int(goal[1])][int(goal[0])] == 255):
          occupied = False
          self.ax.plot(goal[0], goal[1], '.b')
        else:
          print 'Cannot place a goal point there'
          self.ax.set_xlabel('Cannot place a goal point there, choose another point')

      #ppl.draw()
      return start, goal

def main():
    rrt = RRT()
    print("Starting RRT Node ...")
    rospy.init_node('RRT_Node', anonymous=True)

    try:
        # while not rospy.is_shutdown():
        #     rrt.draw()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()





if __name__ == '__main__':
    main()
