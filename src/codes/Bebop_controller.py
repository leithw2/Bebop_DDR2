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

class Reader():
    def __init__(self):
        self.pose_pub = rospy.Publisher('/bebop/command/control', Pose,queue_size=1)
        #self.image_sub = rospy.Subscriber("/cv_camera/image_raw", Image,self.camera_callback)
        self.image_sub = rospy.Subscriber("/bebop/camera1/image_raw", Image,self.camera_callback)
        self.pose_sub = rospy.Subscriber("/bebop/pose", Pose,self.pose_callback)

        self.bridge = CvBridge()
        self.rate = rospy.Rate(30)
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
        self.stateMachine = 0
        ##############################
        self.set_stateMachine(1)

        self.error = 0

        # self.new_pos(12,-9,0)
        # self.new_pos(12,0,0)
        # self.new_pos(7,0,0)
        # self.new_pos(7,-9,0)
        # self.new_pos(3,-9,0)
        self.new_pos(-5,5,3,-0)

        #Define Tag Properties
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters =  aruco.DetectorParameters_create()

        # define range of blue color in HSV
        self.lower_red_1 = np.array([0,150,50])
        self.upper_red_1 = np.array([10,255,255])
        self.lower_red_2 = np.array([170,150,50])
        self.upper_red_2 = np.array([179,255,255])

        #init first target location
        #self.move_to(self.positions)
    def new_pos(self, x,y,z,theta):
        orientation = R.from_euler('z', theta, degrees=True).as_quat()
        self.positions.append(Pose(Point(x,y,z), Quaternion(orientation[0],orientation[1],orientation[2],orientation[3])))


    def set_stateMachine(self, state):
        self.stateMachine = state

    def get_stateMachine(self):
        return self.stateMachine

    def set_target_pose(self, pose):
        self.target_pose = pose

    def get_target_pose(self):
        return self.target_pose

    def set_actual_pose(self, pose):
        self.actual_pose = pose

    def get_actual_pose(self):
        return self.actual_pose

    def set_stateMachine(self, state):
        self.stateMachine = state

    def get_stateMachine(self):
        return self.stateMachine

    def get_colorFound_orientation(self):
        return self.colorFound_orientation

    def set_colorFound_orientation(self, orientation):
        self.colorFound_orientation = orientation

    def move_to(self):
        self.set_target_pose(self.positions.pop())
        print("Go to next position :", self.target_pose.position)
        self.send()
        return self.get_target_pose()
        #if self.get_stateMachine() == 2:
        #    self.positions.append(Pose(Point(10, 10, 10), Quaternion(self.get_colorFound_orientation())))
    def move_one(self):
        self.set_target_pose(self.positions[-1])
        print("Go to next position :", self.target_pose.position)
        self.send()
        return self.get_target_pose()
        #if self.get_stateMachine() == 2:
        #    self.positions.append(Pose(Point(10, 10, 10), Quaternion(self.get_colorFound_orientation())))

    def log_state(self, message = ""):
        if self.tic % 60 == 0:
            print("estado actual ", self.get_stateMachine())
            print(message)

    def calc_error_pose(self, actual_pose, target_pose):
        self.error_pose.position.x =  abs(actual_pose.position.x - target_pose.position.x)
        self.error_pose.position.y =  abs(actual_pose.position.y - target_pose.position.y)
        self.error_pose.position.z =  abs(actual_pose.position.z - target_pose.position.z)

        self.error_pose.orientation.x =  abs(actual_pose.orientation.x - target_pose.orientation.x)
        self.error_pose.orientation.y =  abs(actual_pose.orientation.y - target_pose.orientation.y)
        self.error_pose.orientation.z =  abs(actual_pose.orientation.z - target_pose.orientation.z)
        self.error_pose.orientation.w =  abs(actual_pose.orientation.w - target_pose.orientation.w)

        return self.error_pose.position.x + self.error_pose.position.y + self.error_pose.position.z + self.error_pose.orientation.x + self.error_pose.orientation.y + self.error_pose.orientation.z + self.error_pose.orientation.w

    def get_error_pose(self):
        return self.error_pose

    def update(self, callback):
        self.tic += 1
        self.log_state()
        if callback.pose == callback:
            if self.get_stateMachine() == 0:
                pass

            if self.get_stateMachine() == 1:
                if len(self.positions) > 0:
                    print("Move_to = ", self.move_to() )
                    #print ("error: ", self.get_error_pose())
                    self.set_stateMachine(2)

                else:
                    self.log_state("ya no hay puntos")
                    self.set_stateMachine(7)

            if self.get_stateMachine() == 2:
                if self.is_move_end():
                    self.set_stateMachine(1)

                else:

                    pass
            #Se despaza sin buscar ################# TEST
            if self.get_stateMachine() == 5:
                print("Move_to = ", self.move_one() )
                self.set_stateMachine(6)

            if self.get_stateMachine() == 6:
                #print("target:::::: = ", self.get_target_pose() )
                self.tagReader(self.frame)
                if self.is_move_end():
                    self.set_stateMachine(2)

        if callback.camera == callback:
            #print("callback_camera")
            if self.get_stateMachine() == 2:
                self.colorFinder(self.frame)
                self.log_state("buscando...")

            if self.get_stateMachine() == 7:
                self.tagReader(self.frame)
                self.log_state("target encotrado...")

            if self.get_stateMachine() == 4:
                row,cols,channels = self.frame.shape
                xc,yc = self.colorFinder(self.frame)
                center = float(cols)/2

                print("detectado... x=", (xc  - center) / center, " y=", yc)
                speed = -5.0

                delta =  (xc  - center) / center
                self.positions = []

                self.new_pos(self.get_actual_pose().position.x + .5 ,self.get_actual_pose().position.y + delta * speed,0)
                print(self.get_actual_pose().position.x + .5 ,self.get_actual_pose().position.y  + delta * speed,0)
                self.set_stateMachine(5)


            cv2.imshow('Contornos',self.video_output)
            cv2.waitKey(2)

    def is_move_end(self):
        self.send()
        error = self.calc_error_pose(self.get_actual_pose(), self.get_target_pose())
        print(error)
        if error <= .3:
            print("arrive to: ",len(self.positions) )
            #print( self.get_actual_pose())
            return True
        else:
            #
            return False

    def pose_callback(self, pose):
        self.set_actual_pose(pose)
        self.update(callback.pose)

    def odometry_callback(self,data):

        pass

    def camera_callback(self,data):

        global i
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.video_output = self.frame
    	except CvBridgeError as e:
    		print(e)

        self.update(callback.camera)


    def send(self):
        self.rate.sleep()
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
                self.set_stateMachine(4)
                break
        return xc,yc



    def tagReader(self, frame):

        imgray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        #corners, ids, rejectedImgPoints = aruco.detectMarkers(imgray, self.aruco_dict, parameters=self.parameters)
        #frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

        if ids is not None:

            for i in range(len(ids)):
                print(corners[i][0])

            # display the result
            self.video_output = frame_markers
            self.set_stateMachine(7)
            return True
        return False



if __name__ == '__main__':
    rospy.init_node('tag_node', anonymous=False)
    rospy.loginfo('ProgramSTarted_tag_node')
    try:
        reader = Reader()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
    cv2.destroyAllWindows()
