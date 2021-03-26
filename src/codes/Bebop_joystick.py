import numpy as np

import matplotlib.pyplot as plt
import matplotlib as mpl
import rospy
import cv2
import numpy
#from cv2 import aruco
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



if __name__ == '__main__':
    rospy.init_node('Joystick_node', anonymous=False)
    rospy.loginfo('ProgramSTarted_Joystick_node')
    try:
        joystick = Joystick()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
    cv2.destroyAllWindows()
