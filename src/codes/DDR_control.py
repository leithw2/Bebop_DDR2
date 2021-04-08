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
from geometry_msgs.msg import Transform, Quaternion, Point, Twist, Pose
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge, CvBridgeError


class controller():
    def __init__(self):
        self.haveOdom = False
        self.cmd_vel_pub = rospy.Publisher('/roboto_diff_drive_controller/cmd_vel', Twist,queue_size=1)
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image,self.camera_callback)
        self.odom_sub = rospy.Subscriber("/roboto_diff_drive_controller/odom", Odometry,self.odometry_callback)

        self.bridge = CvBridge()
        self.r = rospy.Rate(50)

        self.positions = []
        self.positions.append(Pose(Point(0,0,0), Quaternion(0,0,0,0)))


        self.phi = np.pi*0/2 #orientation respect of x axle
        self.L = 0.33 #Distance between wheels
        self.R = 0.05
        self.ts = 0
        self.time = rospy.Time.now().to_sec()
        #Expected point to be reached
        self.xrt = 0 #Type here the desired x value
        self.yrt = 0#Type here the desired y value

        self.pose = Odometry().pose.pose.position

        self.xrd = self.xrt
        self.yrd = self.yrt


        self.a = 0.0 #distance from the center of wheels til the control

        self.u = 0
        self.w = 0
        #initial coordenates of the robot in funtion of control localization

        # self.xr = self.xc + self.a * np.cos(self.phi)
        # self.yr = self.yc + self.a * np.sin(self.phi)

        #self.search_next(self.positions)

        return


    def search_next(self, positions):

        if positions:
            #self.target_pose = self.positions.pop()
            #print(self.target_pose)
            self.move()



    def move(self):

        xre = self.xrd - self.pose.x
        yre = self.yrd - self.pose.y

        #Error matrix (2,1)target
        e = np.array([xre, yre])
        #print e
        #Jacobian matrix (2,2)
        J = np.array([[np.cos(self.phi), -np.sin(self.phi)],
                      [np.sin(self.phi),  np.cos(self.phi)]])

        #Matrix of gaining parameters (2,2)
        K = np.array([[.5, 0.0],
                      [0.0, .1]]);

        #control law
        v =  np.linalg.inv(J).dot(K).dot(e);

        #initial lineal and angular velocities of the robot
        u = v[0];
        w = v[1];
        #print ("target pose:::::::::::::::.",self.pose)
        print ("target pose error ", xre," ",yre  )

        #print (v)
        self.send_u_w(u, w)


        #print ("Velocidad lineal = ", u)
        #print ("Velocidad angular = ", w)

        #data return
        #u = self.u
        #w = self.w

        #control actions on the robot
        #xrp = self.u controller position * np.cos(self.phi) - self.a * self.w * np.sin(self.phi);
        #yrp = self.u * np.sin(self.phi) + self.a * self.w * np.cos(self.phi);
        #print (xrp, yrp)
        #Positions
        # self.xr  = self.xr  + self.ts * xrp;
        # self.yr  = self.yr  + self.ts * yrp;
        self.phi = self.phi + self.ts * self.w;
        #print (self.phi)
        #xc(k+1)=xr(k+1)-a*cos(phi(k+1));
        #yc(k+1)=yr(k+1)-a*sin(phi(k+1));

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


    def odometry_callback(self, data):

        #print self.ts
        #self.haveOdom = True
        self.actual_pose = data
        self.u = data.twist.twist.linear.x
        self.w = data.twist.twist.angular.z
        self.pose = data.pose.pose.position
        r = [data.pose.pose.orientation.x,
              data.pose.pose.orientation.y,
              data.pose.pose.orientation.z,
                                           1]
        r = R.from_quat(r, normalized = False)
        print(r.as_euler('xyz',degrees=False)[2])
        #self.phi = r.as_euler('xyz',degrees=False)[2]

        #self.xrd = self.xrt - self.pose.x
        #self.yrd = self.yrt - self.pose.y
        self.move()
        #self.ts = rospy.Time.now().to_sec() - self.time
        self.r.sleep()
        self.ts = rospy.Time.now().to_sec() - self.time
        self.time = rospy.Time.now().to_sec()



    def camera_callback(self,data):
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
