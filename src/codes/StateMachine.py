import numpy as np
import rospy
from std_msgs.msg import String


class state1():
    def __init__(self):
        self.name="1"
    def enter(self):
        self.some = 0

    def do(self):
        self.some = 0
        return 1
    def exit(self):
        self.some = 0

class state2():
    def __init__(self):
        self.name="2"
    def enter(self):
        self.some = 0

    def do(self):
        self.some = 0
        return 2
    def exit(self):
        self.some = 0



class SM():
    def __init__(self):
        self.state_pub = rospy.Publisher('/SM/state', String, queue_size=1)
        self.bebop_state_pub = rospy.Subscriber('/Bebop/state', String, self.bebop_callback)
        self.ddr_state_pub = rospy.Subscriber('/ddr/state', String, self.ddr_callback)
        #self.image_sub = rospy.Subscriber("/cv_camera/image_raw", Image,self.camera_callback)

        self.rate = rospy.Rate(30)
        self.tic = 0
        self.state = 0
        ###############################
        self.stateMachine = 0
        ##############################
        self.set_stateMachine(1)
        self.bebop_state = 0
        self.ddr_state = 0

    def set_stateMachine(self, state):
        self.stateMachine = state

    def get_stateMachine(self):
        return self.stateMachine


    def log_state(self, message = ""):
        if self.tic % 60 == 0:
            print("estado actual ", self.get_stateMachine())
            print(message)

    def update(self):
        msg = "" + str(self.bebop_state) + " " + str(self.ddr_state)
        print(msg)
        self.send(msg)

    def send(self, msg):
        self.rate.sleep()
        try:
            self.state_pub.publish(msg)
        except rospy.ROSInterruptException as ros_e :
            print(ros_e)

    def bebop_callback(self, data):
        self.bebop_state = data.data

    def ddr_callback(self, data):
        self.ddr_state = data.data


if __name__ == '__main__':
    rospy.init_node('SM_node', anonymous=False)
    rospy.loginfo('ProgramSTarted_SM_node')
    try:
        sm = SM()
        while not rospy.is_shutdown():
            sm.update()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
