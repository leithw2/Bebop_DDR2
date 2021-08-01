import numpy as np
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy.spatial.transform import Rotation as R
import rospy
import tf2_ros
import tf2_geometry_msgs
import cv2
import numpy
import std_msgs.msg
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import Transform, Quaternion, Point, Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge, CvBridgeError


class controller():
    def __init__(self):
        self.havePath = False
        self.haveDronePath = False
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.bridge = CvBridge()
        self.r = rospy.Rate(60)
        self.phi = np.pi*0/2 #orientation respect of x axle
        self.ts = 0
        self.time = rospy.Time.now().to_sec()
        self.pose = Odometry().pose.pose.position
        self.u = 0
        self.w = 0
        self.moveState = 0
        self.stateMachine = 0
        self.bebop_state = 0
        self.positions = []
        self.path = []
        self.drone_path =[]
        self.target_pose =[]

        self.cmd_vel_pub = rospy.Publisher('/roboto_diff_drive_controller/cmd_vel', Twist,queue_size=1)

        #self.goal_pose_sub= rospy.Subscriber('/goal/pose', Point, self.goal_pose_callback)
        self.bebop_state_sub= rospy.Subscriber('/bebop/state', String, self.bebop_state_callback)
        #self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image,self.camera_callback)
        self.odom_sub = rospy.Subscriber("/roboto_diff_drive_controller/odom", Odometry,self.odometry_callback)
        self.path_sub = rospy.Subscriber("/rrt/path", Path,self.path_callback)
        self.path_sub = rospy.Subscriber("/rrt/path_drone", Path,self.drone_path_callback)
        self.kinect_scan_sub = rospy.Subscriber("/kinect_scan", LaserScan, self.laserScan_callback)

    def search_next(self, positions):
        print(self.target_pose)
        if self.stateMachine == 4:
            self.moveState = 0
            self.send_u_w(0,0)
            print("Breaks")
            self.stateMachine = 1

        if self.moveState != 3 :
            if self.moveState == 0 and positions :
                #self.target_pose = positions #Debugging
                self.target_pose = positions.pop(0)

                self.moveState = 1

            if self.moveState == 1:
                self.rotate(self.target_pose)
                print("Rotating")

            if self.moveState == 2:
                self.move(self.target_pose)
                print("Moving")
        else:
            self.moveState = 3
            #self.stateMachine = 0 #stand by
            print("Move finish")
        pass

    def rotate(self, target):

        point =  np.array([self.pose.x,self.pose.y])
        dist = numpy.linalg.norm(point - target)
        w = 0
        theta = np.arctan2(target[1] - self.pose.y , target[0]- self.pose.x)
        vel = .1

        if dist <= 0.1:
            self.moveState = 2
            return

        dx = target[0] - self.pose.x
        dy = target[1] - self.pose.y
        target_angle = np.arctan2(dy, dx)
        d_angle = target_angle - self.phi

        if d_angle < np.pi:
            d_angle = d_angle + (np.pi * 2)

        if d_angle > np.pi:
            d_angle = d_angle - (np.pi * 2)

        if abs(d_angle) < 0.05:
           self.moveState = 2

        elif d_angle < 0:
           w = -vel

        else:
           w = +vel

        u = 0
        self.send_u_w(u, w)

    def move(self, target):

        point =  np.array([self.pose.x,self.pose.y])
        dist = numpy.linalg.norm(point - target)
        w = 0
        u = 0
        vel = .1

        if self.moveState == 2 and dist < 0.2:
            self.moveState = 0
            #self.stateMachine = 0 #stand by
            return

        if dist >= 0.1:
            u = vel * dist + .1

        self.send_u_w(u, w)

    def send_u_w(self, u, w):
        command = Twist()
        command.linear.x = u
        command.linear.y = 0
        command.linear.z = 0
        command.angular.x = 0
        command.angular.y = 0
        command.angular.z = w

        self.r.sleep()
        self.cmd_vel_pub.publish(command)

    def path_callback(self, data):

        if self.stateMachine == 1: #following own path
            self.path = []

            for pose in data.poses:
                point = [pose.pose.position.x, pose.pose.position.y]
                self.path.append(point)

            #print (self.positions)
            self.havePath = True

    def drone_path_callback(self, data):

        if self.stateMachine != 1: #following drone path
            self.drone_path = []

            for pose in data.poses:
                point = [-pose.pose.position.y, -pose.pose.position.x]
                self.drone_path.append(point)

            #print (self.positions)
            self.haveDronePath = True
            self.stateMachine = 2


    def laserScan_callback(self, data):
        ranges = data.ranges
        if any(range <= 0.3 for range in ranges):


            if self.stateMachine == 2: #drone guided
                self.stateMachine = 1 #following path
                print("change to following path")
                return
            # if self.stateMachine == 1:#following path
            #     self.positions = []
            #     self.stateMachine = 4 #stand by
            #     print("Fail.........")
            #     return

            if self.stateMachine == 0:
                print("manual mode Fail.........")
                return


    def odometry_callback(self, data):
        #self.r.sleep()
        if self.stateMachine == 1: #following path
            if self.havePath:
                self.positions = self.path
                self.u = data.twist.twist.linear.x
                self.w = data.twist.twist.angular.z
                posestamped = self.tf_trasn(data.pose.pose, "map", "odom")
                self.pose = posestamped.pose.position
                orientation = posestamped.pose.orientation
                r = [orientation.x,
                      orientation.y,
                      orientation.z,
                      orientation.w]
                r = R.from_quat(r, normalized = True)
                self.phi = r.as_euler('xyz',degrees=False)[2]
                self.search_next(self.positions)
            else:
                print("waiting for path.....")

        if self.stateMachine == 2: # Drone guided
            if self.haveDronePath:
                self.positions = self.drone_path
                self.u = data.twist.twist.linear.x
                self.w = data.twist.twist.angular.z
                self.pose = data.pose.pose.position
                orientation = data.pose.pose.orientation
                r = [orientation.x,
                      orientation.y,
                      orientation.z,
                      orientation.w]
                r = R.from_quat(r, normalized = True)
                self.phi = r.as_euler('xyz',degrees=False)[2]
                self.search_next(self.positions)

            self.ts = rospy.Time.now().to_sec() - self.time
            self.time = rospy.Time.now().to_sec()

    def bebop_state_callback(self, data):
        self.bebop_state = int(data.data)


    #def goal_pose_callback(self, data):
        # if self.bebop_state == 6 and self.stateMachine == 0:
        #     self.positions = []
        #     self.positions.append([data.x, data.y])
        #     self.stateMachine = 2 #drone guided
            #print("goal pose callback")

    def camera_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
        	print(e)

        (row,cols,channels) = cv_image.shape
        im = cv_image
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(im.copy(), corners, ids)
        cv2.imshow("img", frame_markers)
        cv2.waitKey(1)

    def tf_trasn(self, pose, to, from_):
        try:
            trans = self.tfBuffer.lookup_transform(to, from_, rospy.Time(), rospy.Duration(1.0))
            ps = PoseStamped()
            ps.pose = pose
            return tf2_geometry_msgs.do_transform_pose(ps, trans)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass


if __name__ == '__main__':
    rospy.init_node('DDRControl', anonymous=False)
    rospy.loginfo('ProgramSTarted_DDRControl')
    try:
        controller = controller()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
    cv2.destroyAllWindows()
