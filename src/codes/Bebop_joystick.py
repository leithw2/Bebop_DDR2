#!/usr/bin/env python
import numpy as np

import matplotlib.pyplot as plt
import matplotlib as mpl
import rospy
import cv2
import numpy
from cv2 import aruco
from cv_bridge import CvBridge, CvBridgeError
import std_msgs.msg
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image
from geometry_msgs.msg import Transform, Quaternion, Point, Twist, Pose
from nav_msgs.msg import Odometry
from enum import Enum

class callback(Enum):
    pose = 0
    camera = 1



class Joystick():
    def __init__(self):
        self.pose_pub = rospy.Publisher('/bebop/command/control', Pose,queue_size=1)
        self.pose_sub = rospy.Subscriber("/bebop/pose", Pose,self.pose_callback)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image,self.camera_callback)
        self.image_pub = rospy.Publisher("/debug/image",Image, queue_size=1)

        self.bridge = CvBridge()
        self.rate = rospy.Rate(60)
        self.tic = 0

        self.auto = 1
        self.state = 0
        self.target_pose = Pose()
        self.actual_pose = Pose()
        self.error_pose = Pose()
        self.positions = []
        self.video_output = []
        self.frame = []
        self.imgray = []
        self.colorFound_orientation=[]
        ###############################
        #Define Tag Properties
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters =  aruco.DetectorParameters_create()

        self.error = 0

        while not rospy.is_shutdown():
            hello_str = "input nextpose %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            x,y,z,psi = input("x,y,z,psi :")
            aposition = self.get_actual_pose().position

            self.new_pos(x + aposition.x ,y + aposition.y, z + aposition.z ,psi)
            self.set_target_pose(self.positions.pop())
            self.send()
            self.rate.sleep()

    def set_actual_pose(self, pose):
        self.actual_pose = pose

    def get_actual_pose(self):
        return self.actual_pose

    def pose_callback(self, pose):
        self.set_actual_pose(pose)

    def new_pos(self, x,y,z,theta):
        orientation = R.from_euler('z', theta, degrees=True).as_quat()
        self.positions.append(Pose(Point(x,y,z), Quaternion(orientation[0],orientation[1],orientation[2],orientation[3])))

    def set_target_pose(self, pose):
        self.target_pose = pose

    def get_target_pose(self):
        return self.target_pose


    def send(self):
        self.rate.sleep()
        try:
            self.pose_pub.publish(self.target_pose)
        except rospy.ROSInterruptException as ros_e :
            print(ros_e)

    def tagReader(self, frame):

        imgray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(imgray, self.aruco_dict, parameters=self.parameters)
        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

        if ids is not None:

            for i in range(len(ids)):

                #print(corners[i][0])
                corner1 ,corner2, corner3, corner4 = corners[i][0]
                height = int(np.sqrt((corner1[0] - corner2[0])**2 + (corner1[1] - corner2[1])**2))
                width = int(np.sqrt((corner3[0] - corner4[0])**2 + (corner3[1] - corner4[1])**2))
                font                   = cv2.FONT_HERSHEY_SIMPLEX
                bottomLeftCornerOfText = (10,500)
                fontScale              = 1
                fontColor              = (255,255,255)
                lineType               = 2




                #m = (corner2[1] - corner3[1]) / (corner2[0] - corner3[0])
                angle = np.arctan2(corner2[1] - corner3[1], corner2[0] - corner3[0] ) + (np.pi*2/2)

                string = "tamano en pixeles H, L, theta " +str(height) + " " + str(width)  + " " + str(angle)

                cv2.putText(frame_markers,string,
                    bottomLeftCornerOfText,
                    font,
                    fontScale,
                    fontColor,
                    lineType)

            # display the result
            self.video_output = frame_markers
            #self.set_stateMachine(7)
            #send_image(self.video_output)
            return True
        return False

    def send_image(self, image):

        image_message = self.bridge.cv2_to_imgmsg(image, "bgr8")
        try:
            self.image_pub.publish(image_message)
        except rospy.ROSInterruptException as ros_e :
            print(ros_e)

        self.rate.sleep()

    def camera_callback(self, data):

        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.video_output = self.frame

        except CvBridgeError as e:
    		print(e)


        self.tagReader(self.frame)
        self.send_image(self.video_output)
        return

if __name__ == '__main__':
    rospy.init_node('Joystick_node', anonymous=False)
    rospy.loginfo('ProgramSTarted_Joystick_node')
    try:
        joystick = Joystick()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
    cv2.destroyAllWindows()
