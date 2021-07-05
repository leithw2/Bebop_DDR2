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
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import Transform, Quaternion, Point, Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge, CvBridgeError


class controller():
    def __init__(self):
        self.haveOdom = False
        self.havePath = False
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.cmd_vel_pub = rospy.Publisher('/roboto_diff_drive_controller/cmd_vel', Twist,queue_size=1)

        self.goal_pose_sub= rospy.Subscriber('/goal/pose', Point, self.goal_pose_callback)
        self.bebop_state_sub= rospy.Subscriber('/bebop/state', String, self.bebop_state_callback)
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image,self.camera_callback)
        self.odom_sub = rospy.Subscriber("/roboto_diff_drive_controller/odom", Odometry,self.odometry_callback)
        self.path_sub = rospy.Subscriber("/rrt/path", Path,self.path_callback)

        self.bridge = CvBridge()
        self.r = rospy.Rate(60)

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

        self.state = 4
        self.bebop_state = 0

        #Debugging
        #self.havePath = True
        self.positions = []

        return


    def search_next(self, positions):

        #positions = (-10,-3) #Debugging
        if self.state != 3:

            if self.state == 0 and positions :
                #self.target_pose = positions #Debugging
                self.target_pose = positions.pop(0)
                self.state = 1

            #print(self.target_pose)
            #print([self.pose.x,self.pose.y])
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
        theta = np.arctan2(target[1] - self.pose.y , target[0]- self.pose.x)

        vel = .1

        if dist <= 0.1:
            self.state = 2
            return


        dx = target[0] - self.pose.x
        dy = target[1] - self.pose.y
        target_angle = np.arctan2(dy, dx)
        #print(target_angle)
        d_angle = target_angle - self.phi
        if d_angle < np.pi:
            d_angle = d_angle + (np.pi * 2)
        if d_angle > np.pi:
            d_angle = d_angle - (np.pi * 2)
        #print(d_angle)
        #d_angle = wrap_to_p(d_angle)    # wrap the angle here
        if abs(d_angle) < 0.05:
           self.state = 2
        elif d_angle < 0:
           w = -vel
        else:
           w = +vel

        u = 0

        self.send_u_w(u, w)


        return

    def move(self, target):

        point =  np.array([self.pose.x,self.pose.y])
        #target = np.array([self.xrd,self.yrd ])
        dist = numpy.linalg.norm(point - target)
        w = 0
        u = 0
        vel = .3

        if self.state == 2 and dist < 0.2:
            self.state = 0

            return

        if dist >= 0.1:
            u = vel * dist + .1



        self.send_u_w(u, w)

        return

    def send_u_w(self, u, w):
        command = Twist()
        command.linear.x = u
        command.linear.y = 0
        command.linear.z = 0
        command.angular.x = 0
        command.angular.y = 0
        command.angular.z = w

        self.cmd_vel_pub.publish(command)

        return

    def path_callback(self, data):
        #print (data.poses)
        self.positions = []

        for pose in data.poses:

            point = [pose.pose.position.x, pose.pose.position.y]
            self.positions.append(point)

        #print (self.positions)
        self.havePath = True
        self.state = 0

    def example_function(self, pose, to, from_):

        try:
            #print("trying..")
            trans = self.tfBuffer.lookup_transform(to, from_, rospy.Time(), rospy.Duration(1.0))
            #print("trying.. end")
            #print(trans)
            ps = PoseStamped()
            ps.pose = pose
            return tf2_geometry_msgs.do_transform_pose(ps, trans)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def odometry_callback(self, data):
        self.r.sleep()
        if self.havePath:

            self.actual_pose = data
            self.u = data.twist.twist.linear.x
            self.w = data.twist.twist.angular.z
            posestamped = self.example_function(data.pose.pose, "map", "odom")

            self.posestamped = posestamped.pose
            self.pose = posestamped.pose.position

            r = [self.posestamped.orientation.x,
                  self.posestamped.orientation.y,
                  self.posestamped.orientation.z,
                  self.posestamped.orientation.w]
            r = R.from_quat(r, normalized = True)
            #print(r.as_euler('xyz',degrees=False)[2])
            self.phi = r.as_euler('xyz',degrees=False)[2]


            self.search_next(self.positions)

        else:
            self.actual_pose = data
            self.u = data.twist.twist.linear.x
            self.w = data.twist.twist.angular.z

            self.pose = data.pose.pose.position
            orientation = data.pose.pose.orientation
            r = [orientation.x,
                  orientation.y,
                  orientation.z,
                  orientation.w]
            r = R.from_quat(r, normalized = True)
            #print(r.as_euler('xyz',degrees=False)[2])
            self.phi = r.as_euler('xyz',degrees=False)[2]

            self.search_next(self.positions)


        self.ts = rospy.Time.now().to_sec() - self.time
        self.time = rospy.Time.now().to_sec()

    def bebop_state_callback(self, data):
        self.bebop_state = int(data.data)
        if data.data == "6":
            print("start_ moving_hovering")

    def goal_pose_callback(self, data):

        if self.bebop_state == 6 and self.state == 4:
            self.positions = []
            self.positions.append([data.x, data.y])
            self.state = 0
            #self.search_next(self.positions)
            print("goal pose callback")

    def camera_callback(self, data):

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
