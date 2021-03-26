#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import tensorflow as tf
from tensorflow import keras
from sklearn.model_selection import train_test_split
import numpy as np
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import time
from numpy import savetxt
from numpy import save
from numpy import load

i=0
reconstructed_model = keras.models.load_model("/home/laptop/codes/modelotest.pb")
class image_converter:
    global i

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callback)
        self.fig=None
        self.model=None


    def callback(self,data):

        global i
        if self.model == None:
            self.model = keras.models.load_model("/home/laptop/codes/modelotest.pb")


        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='32FC1')

        except CvBridgeError as e:
            print(e)

        cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
        c_open = 1
        l_range = 6
        norma = np.linalg.norm([l_range, c_open, c_open])
        #cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 1, 0, cv2.NORM_L2)
        cv_image_norm = cv_image_array / norma
        resized = cv2.resize(cv_image_norm, (128,72) , interpolation = cv2.INTER_AREA)

        resized[np.isnan(resized)]=-1
        #resized = resized.astype('float64')
        test2=np.resize(resized,(72,128))
        test=np.reshape(resized,(1,72,128,1))

        #print(test2)
        #cv2.imshow("imaget 2",self.imaget_x)
        cv2.imshow("128 72",cv_image_norm)
        #plt.imshow(test2)
        #plt.show()
        #test=np.nan_to_num(test)
        #print(resized)
        if self.fig == None:
            plt.ion()
            plt.show()
            self.fig = plt.figure()
            self.ax = self.fig.add_axes([0,0,1,1])




        start_time = time.time()
        res=self.model.predict(test)
        #print(np.round(res,2))
        #print("Tiempo: ", time.time() - start_time)
        cat = np.argmax(res)

        langs = np.arange(6)
        labels = ['0', '1', '2', '3', '4']
        students = res[0]
        x = np.arange(len(labels))
        #print(students)
        self.ax.set_ylabel('Scores')
        self.ax.set_title('Scores by group and gender')
        self.ax.set_xticks(x)
        self.ax.set_xticklabels(labels)
        #self.ax.bar(langs,students)

        result = np.around(res[0],decimals=2)

        x = np.arange(len(labels))  # the label locations
        width = 0.35  # the width of the bars

        rects1 = self.ax.bar(x - width/2, result, width, label='pred')
        # Add some text for labels, title and custom x-axis tick labels, etc.
        self.ax.set_ylabel('Scores')
        self.ax.set_title('Scores by group and gender')
        self.ax.set_xticks(x)
        self.ax.set_xticklabels(labels)
        self.ax.legend()


        def autolabel(rects):
            """Attach a text label above each bar in *rects*, displaying its height."""
            for rect in rects:
                height = rect.get_height()
                self.ax.annotate('{}'.format(height),
                            xy=(rect.get_x() + rect.get_width() / 2, height),
                            xytext=(0, 3),  # 3 points vertical offset
                            textcoords="offset points",
                            ha='center', va='bottom')


        autolabel(rects1)

        self.fig.tight_layout()


        plt.pause(0.001)
        plt.cla()

        if cat == 0 :
            print("pasillo abierto")

        if cat == 1 :
            print("pasillo cerrado")

        if cat == 2 :
            print("vuelta izquierda")

        if cat == 3 :
            print("vuelta derecha")

        if cat == 4 :
            print("espacio abierto")


        cv2.waitKey(1)
        #rospy.sleep(0.1)


def main(args):
    global reconstructed_model
    reconstructed_model = keras.models.load_model("/home/laptop/codes/modelotest.pb")
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

# def evaluar(test):
#     global reconstructed_model
#     #reconstructed_model = keras.models.load_model("/home/laptop/codes/modelotest.pb")
#     res=reconstructed_model.predict(test)
#     reconstructed_model.backend.clear_session()
#     print(np.round(res,2))


if __name__ == '__main__':
    main(sys.argv)
