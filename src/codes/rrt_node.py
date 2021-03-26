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
from PIL import Image as im



class RRT:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/rtabmap/grid_map",OccupancyGrid,self.callback)


    def callback(self,data):
        #print(data)
        map = np.array(data.data)
        width = data.info.width
        height = data.info.height
        resize = np.uint8(np.resize(map, [height,width]))

        resize = 255 - resize

        img= im.fromarray(resize)
        img.save('gfg_dummy_pic.png')
        #cv2.imshow("",grayimg)
        cv2.imshow("",resize)
        cv2.waitKey(1)
        #print(np.array(data.data).shape)
        pass

def main(args):
    rrt = RRT()
    print("Starting RRT Node ...")
    rospy.init_node('RRT_Node', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
