import rospy
import tf
import math
import numpy as np
import cv2
import std_msgs.msg
from geometry_msgs.msg import Transform, Quaternion, Point, Twist, Pose
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Trajectory():
    def __init__(self):
        self.traj_pub = rospy.Publisher('/bebop/command/trajectory', MultiDOFJointTrajectory,queue_size=1)
        self.pose_sub = rospy.Subscriber("/bebop/command/control", Pose,self.pose_callback)
        self.traj = MultiDOFJointTrajectory()
        self.r = rospy.Rate(60)
        self.pose = Pose()
        return

    def pose_callback(self,data):
        self.pose = data
        rospy.loginfo('data get')
        self.update(self.pose)


    def update(self,data):
        self.traj = MultiDOFJointTrajectory()
        self.traj.header.frame_id =''
        self.traj.header.stamp = rospy.Time.now()
        self.traj.joint_names = ["base_link"]

        transforms = Transform()
        transforms.translation.x = data.position.x
        transforms.translation.y = data.position.y
        transforms.translation.z = data.position.z
        transforms.rotation.x = data.orientation.x
        transforms.rotation.y = data.orientation.y
        transforms.rotation.z = data.orientation.z
        transforms.rotation.w = data.orientation.w
        velocities =Twist()
        velocities.linear.x = 100
        velocities.linear.y = 0
        velocities.linear.z = 0
        velocities.angular.x = 0
        velocities.angular.y = 0
        velocities.angular.z = 0

        accelerations=Twist()
        accelerations.linear.x = 100
        accelerations.linear.y = 0
        accelerations.linear.z = 0
        accelerations.angular.x = 0
        accelerations.angular.y = 0
        accelerations.angular.z = 0

        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Duration(0.1))
        self.traj.points.append(point)
        self.traj_pub.publish(self.traj)
        rospy.loginfo('data post')
        self.r.sleep()

        return

if __name__ == '__main__':
    rospy.init_node('Inteface_node', anonymous=False)
    rospy.loginfo('ProgramSTarted')
    trajectory = Trajectory()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
    cv2.destroyAllWindows()
