import numpy as np
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy.spatial.transform import Rotation as R
import rospy
import cv2
import numpy
import std_msgs.msg
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import Transform, Quaternion, Point, Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge, CvBridgeError


class controller():
    def __init__(self):
        self.haveOdom = False
        self.havePath = False
        self.cmd_vel_pub = rospy.Publisher('/roboto_diff_drive_controller/cmd_vel', Twist,queue_size=1)
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image,self.camera_callback)
        self.odom_sub = rospy.Subscriber("/roboto_diff_drive_controller/odom", Odometry,self.odometry_callback)
        self.path_sub = rospy.Subscriber("/rrt/path", Path,self.path_callback)

        self.bridge = CvBridge()
        self.r = rospy.Rate(50)

        self.phi = np.pi*0/2 #orientation respect of x axle
        self.L = 0.33 #Distance between wheels
        self.R = 0.05
        self.ts = 0
        self.time = rospy.Time.now().to_sec()
        self.pose = Odometry().pose.pose.position

        #self.xrd = +5.8
        #self.yrd = -3.12

        self.a = 0.0 #distance from the center of wheels til the control

        self.u = 0
        self.w = 0

        self.state = 0

        return


    def search_next(self, positions):


        if self.state != 3:

            if self.state == 0 and positions :
                self.target_pose = positions.pop(0)
                self.state = 1

            print(self.target_pose)
            print([self.pose.x,self.pose.y])
            if self.state == 1:
                self.rotate(self.target_pose)
                print("Rotating")

            if self.state == 2:
                self.move(self.target_pose)
                print("Moving")
        else:
            self.state = 3
            print("Path finish")
        pass


    def rotate(self, target):

        point =  np.array([self.pose.x,self.pose.y])
        #target = np.array([self.xrd,self.yrd ])

        dist = numpy.linalg.norm(point - target)
        w = 0
        #Error matrix (2,1)target
        angle = np.arctan2(target[1] - self.pose.y , target[0]- self.pose.x)
        vel = .3

        if dist <= 0.1:
            self.state = 2
            return


        if self.state == 1 and abs(angle - self.phi) < 0.1:
            self.state = 2
            return

        # if angle > 0:
        #     if self.phi < angle:
        #         w = vel * abs(angle - self.phi) +.1 #positive
        #
        #     if self.phi >= angle:
        #         w = -vel * abs(angle - self.phi) -.1 #negative
        #
        # if angle <= 0:
        #     if self.phi > angle:
        #         w = -vel * abs(angle - self.phi) - .1 #negative
        #
        #     if self.phi <= angle:
        #         w = vel * abs(angle - self.phi) +.1 #positive

        if angle > 0:
            if self.phi < angle:
                w = +.5 #positive

            if self.phi >= angle:
                w = -.5 #negative

        if angle <= 0:
            if self.phi > angle:
                w = -.5 #negative

            if self.phi <= angle:
                w = +.5 #positive

        #map_angle =
        #map_phi =
        #initial lineal and angular velocities of the robot
        u = 0

        #print ("target pose:::::::::::::::.",self.pose)
        #print ("target pose error ", xre," ",yre  )
        #print ("angle target ", angle," actual angle ", self.phi)
        #print (v)
        self.send_u_w(u, w)


        return

    def move(self, target):

        point =  np.array([self.pose.x,self.pose.y])
        #target = np.array([self.xrd,self.yrd ])
        dist = numpy.linalg.norm(point - target)
        w = 0
        u = 0
        vel = 1

        if self.state == 2 and dist < 0.1:
            self.state = 0

            return

        if dist >= 0.1:
            u = vel * dist + .1



        self.send_u_w(u, w)

        return

    def send_u_w(self, u, w):
        target = Twist()
        target.linear.x = u
        target.linear.y = 0
        target.linear.z = 0
        target.angular.x = 0
        target.angular.y = 0
        target.angular.z = w

        self.cmd_vel_pub.publish(target)

        return

    def path_callback(self, data):
        print (data.poses)
        self.positions = []

        for pose in data.poses:
            point = [pose.pose.position.x, pose.pose.position.y]
            self.positions.append(point)

        print (self.positions)
        self.havePath = True


    def odometry_callback(self, data):

        if self.havePath:

            self.actual_pose = data
            self.u = data.twist.twist.linear.x
            self.w = data.twist.twist.angular.z
            self.pose = data.pose.pose.position
            r = [data.pose.pose.orientation.x,
                  data.pose.pose.orientation.y,
                  data.pose.pose.orientation.z,
                  data.pose.pose.orientation.w]
            r = R.from_quat(r, normalized = True)
            #print(r.as_euler('xyz',degrees=False)[2])
            self.phi = r.as_euler('xyz',degrees=False)[2]


            self.search_next(self.positions)

            self.r.sleep()
            self.ts = rospy.Time.now().to_sec() - self.time
            self.time = rospy.Time.now().to_sec()


    def camera_callback(self, data):
        #print "here"
        global i
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    	except CvBridgeError as e:
    		print(e)
    	(row,cols,channels) = cv_image.shape


    	im = cv_image
        #im = cv2.imread("images/tag.png")
        #im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)

        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(im.copy(), corners, ids)

        #if ids is not None:
            #for i in range(len(ids)):
                #print(corners[i][0])

        # display the result
        cv2.imshow("img", frame_markers)
        cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('DDRControl', anonymous=False)
    rospy.loginfo('ProgramSTarted_DDRControl')
    try:
        controller = controller()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
    cv2.destroyAllWindows()