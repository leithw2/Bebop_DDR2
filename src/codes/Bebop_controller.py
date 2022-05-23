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
from std_msgs.msg import String
from geometry_msgs.msg import Transform, Quaternion, Point, Twist, Pose
from nav_msgs.msg import Odometry
from enum import Enum

class callback(Enum):
    pose = 0
    camera = 1

class Controller:

    def __init__(self):

        #self.image_sub = rospy.Subscriber("/cv_camera/image_raw", Image,self.camera_callback)

        self.bridge         = CvBridge()
        self.rate           = rospy.Rate(60)
        self.rate2          = rospy.Rate(60)
        self.tic            = 0
        #Define Tag Properties
        self.aruco_dict     = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters     = aruco.DetectorParameters_create()

        # define range of blue color in HSV
        self.lower_red_1    = np.array([0,150,50])
        self.upper_red_1    = np.array([10,255,255])
        self.lower_red_2    = np.array([170,150,50])
        self.upper_red_2    = np.array([179,255,255])

        self.auto           = 1
        self.state          = 0
        self.target_pose    = Pose()
        self.actual_pose    = Pose()
        self.error_pose     = Pose()
        self.goal_tag_pose  = []
        self.goal_pose      = [8,-8]
        self.robot_tag_pose = []
        self.positions      = []
        self.video_output   = []
        self.frame          = []
        self.imgray         = []
        self.image_size     = []
        #self.colorFound_orientation=[]
        self.error = 0
        ###############################
        self.stateMachine   = 0
        ##############################
        self.set_stateMachine(1)

        # # self.new_pos( 0, 0,4,0)
        # self.new_pos( 2,-2,4,0)
        # self.new_pos( 0,-2,4,0)
        # self.new_pos( 0,-4,4,0)
        # self.new_pos( 6, 0,4,0)
        # self.new_pos( 0,-6,4,0)
        # self.new_pos( 8,0,4,0)
        # self.new_pos( 0,-8,4,0)
        # self.new_pos( 8,-6,4,0)
        # self.new_pos( 6,-8,4,0)
        self.new_pos( 8,-8,4,0)
        # self.new_pos( 8, 0,4,0)


        self.positions = self.positions[::-1]


        self.image_sub      = rospy.Subscriber("/camera/color/image_raw", Image,self.camera_callback)
        self.pose_sub       = rospy.Subscriber("/bebop/pose", Pose, self.pose_callback)
        self.ddr_state_sub  = rospy.Subscriber('/ddr/state', String, self.ddr_state_callback, queue_size=1, buff_size=2**24)


        self.pose_pub       = rospy.Publisher('/bebop/command/control', Pose,queue_size=1)
        self.image_pub      = rospy.Publisher("/debug/image",Image, queue_size=1)
        self.bebop_state_pub= rospy.Publisher('/bebop/state', String, queue_size=1)
        self.goal_pub       = rospy.Publisher('/bebop/goal', Point, queue_size=1)
        self.start_pub      = rospy.Publisher("/bebop/start",Point, queue_size=1)
        #init first target location
        #self.move_to(self.positions)

    def set_stateMachine(self, state):
        self.stateMachine = state

    def get_stateMachine(self):
        return self.stateMachine

    def set_target_pose(self, pose):
        self.target_pose = pose

    def get_target_pose(self):
        return self.target_pose

    def set_goal_pose(self, pose):
        self.goal_pose = pose

    def get_goal_pose(self):
        return self.goal_pose

    def set_goal_tag_pose(self, pose):
        self.goal_tag_pose = pose

    def get_goal_tag_pose(self):
        return self.goal_tag_pose

    def set_robot_pose(self, pose):
        self.robot_pose = pose

    def get_robot_pos(self):
        return self.robot_pos

    def set_robot_tag_pose(self, position):
        self.robot_tag_pose = position

    def get_robot_tag_pose(self):
        return self.robot_tag_pose

    def set_actual_pose(self, pose):
        self.actual_pose = pose

    def get_actual_pose(self):
        return self.actual_pose

    def get_colorFound_orientation(self):
        return self.colorFound_orientation

    def set_colorFound_orientation(self, orientation):
        self.colorFound_orientation = orientation

    def move_to(self):
        self.set_target_pose(self.positions.pop())
        print("Go to next position :", self.target_pose.position)
        self.send()
        return self.get_target_pose()

    def move_one(self):
        self.set_target_pose(self.positions[-1])
        print("Go to next position :", self.target_pose.position)
        self.send()
        return self.get_target_pose()

    def log_state(self, message = ""):
        #self.rate.sleep()
        if self.tic % 60 == 0:
            #print("estado actual ", self.get_stateMachine())
            self.send()
        if self.get_stateMachine() == 6:
            self.send_goal(self.get_goal_pose())

        if self.tic % 120 == 0 and message != "":
            self.send_state(self.get_stateMachine())
            print(message)

    def calc_error_pose(self, actual_pose, target_pose):

        self.error_pose.position.x =  abs(actual_pose.position.x - target_pose.position.x)
        self.error_pose.position.y =  abs(actual_pose.position.y - target_pose.position.y)
        self.error_pose.position.z =  abs(actual_pose.position.z - target_pose.position.z)

        self.error_pose.orientation.x =  abs(actual_pose.orientation.x - target_pose.orientation.x)
        self.error_pose.orientation.y =  abs(actual_pose.orientation.y - target_pose.orientation.y)
        self.error_pose.orientation.z =  abs(actual_pose.orientation.z - target_pose.orientation.z)
        self.error_pose.orientation.w =  abs(actual_pose.orientation.w - target_pose.orientation.w)
        #print(self.error_pose)

        return self.error_pose.position.x + self.error_pose.position.y + self.error_pose.position.z + self.error_pose.orientation.x + self.error_pose.orientation.y + self.error_pose.orientation.z + self.error_pose.orientation.w

    def get_error_pose(self):
        return self.error_pose

    def new_pos(self, x,y,z,theta):
        orientation = R.from_euler('z', theta, degrees=True).as_quat()
        self.positions.append(Pose(Point(x,y,z), Quaternion(orientation[0],orientation[1],orientation[2],orientation[3])))


    def update(self, callback):
        self.tic += 1
        self.log_state()

        ####debbuger
        #self.send_goal(self.get_goal_pose())
        #########
        p = 0.01
        if callback.pose == callback:
            if self.get_stateMachine() == 0: #stanby
                pass

            if self.get_stateMachine() == 1: # move next
                if len(self.positions) > 0:
                    self.log_state(str("Move_to = ") + str(self.move_to()))
                    self.set_stateMachine(2)

                else:
                    self.log_state("ya no hay puntos")
                    self.set_stateMachine(7) #stop

            if self.get_stateMachine() == 2: #moving
                if self.is_move_end():
                    self.set_stateMachine(1)

            if self.get_stateMachine() == 5: #return
                self.log_state("Returning")
                if self.is_move_end():
                    self.set_stateMachine(6)


        if callback.camera == callback:
            #print("callback_camera")
            if self.get_stateMachine() == 2: #moving
                #self.colorFinder(self.frame)
                if self.tagReader(self.frame,0):
                    self.set_stateMachine(3)
                self.log_state("buscando...")

            if self.get_stateMachine() == 3: #correcting
                #self.tagReader(self.frame)
                self.log_state("target encotrado, alineando...")
                self.tagReader(self.frame, 0)
                if self.tracker(self.get_goal_tag_pose()):
                    self.set_goal_pose([self.get_actual_pose().position.x, self.get_actual_pose().position.y])
                    ######self.set_stateMachine(4)

            if self.get_stateMachine() == 4: #goal found
                #self.tagReader(self.frame)
                self.log_state("coordenadas listas ")
                self.new_pos(0,0,4,0)
                self.move_one()
                self.log_state("Returnin_to = " + str(self.move_to()) )
                self.set_stateMachine(5)

            if self.get_stateMachine() == 6: #trackig_robot
                #self.tagReader(self.frame)
                self.log_state("Tracking")
                #self.log_state(str(self.get_goal_pose()))

                if self.tagReader(self.frame, 1):
                    self.tracker(self.get_robot_tag_pose())



    def is_move_end(self):
        #self.send()
        error = self.calc_error_pose(self.get_actual_pose(), self.get_target_pose())
        #print(error)
        if error <= .1:
            print("arrive to: ",len(self.positions) )
            #print( self.get_actual_pose())
            return True
        else:
            #
            return False

    def tracker(self, tracked_object):
        i       = 0
        p       = 0.01
        speedx  = 0
        speedy  = 0
        delta   = 5

        if tracked_object[0] > (self.image_size[1]/2) + delta:
            speedy = abs(tracked_object[0] - (self.image_size[1]/2)) * -p
            i+=1
            self.sendCommand(0,speedy,0,0)


        if tracked_object[0] < (self.image_size[1]/2) - delta:
            speedy = abs(tracked_object[0] - (self.image_size[1]/2)) * +p
            i+=1
            self.sendCommand(0,speedy,0,0)

        if tracked_object[1] > (self.image_size[0]/2) + delta:
            speedx = abs(tracked_object[1] - (self.image_size[0]/2)) * -p
            i+=1
            self.sendCommand(speedx,0,0,0)

        if tracked_object[1] < (self.image_size[0]/2) - delta:
            speedx = abs(tracked_object[1] - (self.image_size[0]/2)) * +p
            i+=1
            self.sendCommand(speedx,0,0,0)

        if i == 0:

            return True
        else:
            #self.sendCommand(speedx,speedy,0,0)

            return False

    def ddr_state_callback(self, pose):

        pass

    def pose_callback(self, pose):

        self.set_actual_pose(pose)
        self.update(callback.pose)

    def camera_callback(self, data):

        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.video_output = self.frame
            height, width, channels = self.frame.shape
            self.image_size = [height, width]

        except CvBridgeError as e:
            print(e)

        self.update(callback.camera)
        #self.tagReader2(self.frame)#NEW
        self.send_image(self.video_output)
        return

    def send_image(self, image):

        self.rate.sleep()
        image_message = self.bridge.cv2_to_imgmsg(image, "bgr8")
        try:
            self.image_pub.publish(image_message)
        except rospy.ROSInterruptException as ros_e :
            print(ros_e)

    def send_state(self, state):

        self.rate.sleep()
        try:
            self.bebop_state_pub.publish(str(state))
        except rospy.ROSInterruptException as ros_e :
            print(ros_e)

    def send_start(self, p):

        self.rate.sleep()
        point   = Point()
        point.x = p[0]
        point.y = p[1]
        point.z = 0

        try:
            self.start_pub.publish(point)
        except rospy.ROSInterruptException as ros_e :
            print(ros_e)

    def send_goal(self, p):

        self.rate.sleep()
        point   = Point()
        point.x = p[0]
        point.y = p[1]
        point.z = 0

        point.x =  8.07
        point.y = -7.92
        point.z = 0
        try:
            self.goal_pub.publish(point)
        except rospy.ROSInterruptException as ros_e :
            print(ros_e)

    def sendCommand(self,x,y,z,psi):

        self.rate.sleep()
        aposition = self.get_actual_pose().position
        z = 4
        self.new_pos(x + aposition.x ,y + aposition.y, z ,psi)
        self.set_target_pose(self.positions[-1])
        self.send()

        pass

    def send(self):
        self.rate2.sleep()
        try:
            self.pose_pub.publish(self.target_pose)
        except rospy.ROSInterruptException as ros_e :
            print(ros_e)


    def colorFinder(self, frame):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only Red colors
        mask1 = cv2.inRange(hsv, self.lower_red_1, self.upper_red_1)
        mask2 = cv2.inRange(hsv, self.lower_red_2, self.upper_red_2)
        mask = mask1 + mask2
        # Bitwise-AND mask and original image
        #res = cv2.bitwise_and(frame,frame, mask= mask)
        #ret,thresh = cv2.threshold(imgray,127,255,0)
        contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        img = cv2.drawContours(frame.copy(),contours, -1, (255,255,0), 3)

        xc = 0
        yc = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)

            self.log_state(area)
            if area >= 100:
                print("Area = ", area)
                x,y,w,h = cv2.boundingRect(cnt)
                xc = x+w/2
                yc = y+h/2
                self.video_output = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                #print (area)


                #calcular bien los quaterniones, nunca deben ser todos 0
                # r = [self.actual_pose.orientation.x,
                #      self.actual_pose.orientation.y,
                #      self.actual_pose.orientation.z,
                #                                   1]
                # r = R.from_quat(r, normalized = False)
                # print(r.as_euler('xyz',degrees=True))

                self.set_colorFound_orientation(self.actual_pose.orientation)
                #self.set_stateMachine(4)
                break
        return xc,yc



    def tagReader(self, frame, id):

        imgray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(imgray, self.aruco_dict, parameters=self.parameters)
        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        # display the result
        self.video_output = frame_markers

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

                if ids[i] == 0 and ids[i] == id: #taget
                    self.set_goal_tag_pose(corner1)
                    return True

                if ids[i] == 1 and ids[i] == id: #robot
                    self.set_robot_tag_pose(corner1)
                    return True

        return False



if __name__ == '__main__':
    rospy.init_node('tag_node', anonymous=False)
    rospy.loginfo('ProgramSTarted_tag_node')
    try:
        controller = Controller()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
    cv2.destroyAllWindows()
