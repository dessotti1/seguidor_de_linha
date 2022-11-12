#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

class Sensores:

    def __init__(self):

        self.sensores = np.array([-1, -1, -1, -1, -1])
        self.publish_time = 0.05

        self.img_sensor_full_left = rospy.Subscriber("/sensor/camera_full_left/image_raw", Image, self.imgFullLeftCallback, queue_size = 10)
        self.img_sensor_left = rospy.Subscriber("/sensor/camera_left/image_raw", Image, self.imgLeftCallback, queue_size = 10)
        self.img_sensor_center = rospy.Subscriber("/sensor/camera_center/image_raw", Image, self.imgCenterCallback, queue_size = 10)
        self.img_sensor_right = rospy.Subscriber("/sensor/camera_right/image_raw", Image, self.imgRightCallback, queue_size = 10)
        self.img_sensor_full_right = rospy.Subscriber("/sensor/camera_full_right/image_raw", Image, self.imgFullRightCallback, queue_size = 10)   

        self.sensor_pub = rospy.Publisher("/dados", Int32MultiArray, queue_size=10)
        
        self.timer_pub = rospy.Timer(rospy.Duration(self.publish_time), self.timerCallback)   


    def imgFullLeftCallback (self, ros_img):

        img = bridge.imgmsg_to_cv2(ros_img)       
        self.img_full_left = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        self.cor_full_left = np.mean(self.img_full_left)

        if (self.cor_full_left < 150):
            self.sensores[0] = 0

        elif (self.cor_full_left > 230):
            self.sensores[0] = 1
        
        else:
            rospy.logwarn("Cor identificada diferente de branco e preto!\n")

    def imgLeftCallback (self, ros_img):

        img = bridge.imgmsg_to_cv2(ros_img)
        self.img_left = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        self.cor_left = np.mean(self.img_left)

        if (self.cor_left < 150):
           self.sensores[1] = 0

        elif (self.cor_left > 230):
            self.sensores[1] = 1
        
        else:
            rospy.logwarn("Cor identificada diferente de branco e preto!\n")

    def imgCenterCallback (self, ros_img):

        img = bridge.imgmsg_to_cv2(ros_img)
        self.img_center = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        self.cor_center = np.mean(self.img_center)

        if (self.cor_center < 150):
            self.sensores[2] = 0

        elif (self.cor_center > 230):
            self.sensores[2] = 1
        
        else:
            rospy.logwarn("Cor identificada diferente de branco e preto!\n")
        
    def imgRightCallback (self, ros_img):

        img = bridge.imgmsg_to_cv2(ros_img)
        self.img_right = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        self.cor_right = np.mean(self.img_right)

        if (self.cor_right < 150):
            self.sensores[3] = 0

        elif (self.cor_right > 230):
            self.sensores[3] = 1
        
        else:
            rospy.logwarn("Cor identificada diferente de branco e preto!\n")

    def imgFullRightCallback (self, ros_img):

        img = bridge.imgmsg_to_cv2(ros_img)
        self.img_full_right = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        self.cor_full_right = np.mean(self.img_full_right)

        if (self.cor_full_right < 150):
            self.sensores[4] = 0

        elif (self.cor_full_right > 230):
            self.sensores[4] = 1
        
        else:
            rospy.logwarn("Cor identificada diferente de branco e preto!\n")        

    def timerCallback(self, event):
        msg = Int32MultiArray()
        msg.data = self.sensores
        self.sensor_pub.publish(msg)

if __name__ == '__main__':

    try:
        rospy.init_node("sensores")
        Sensores()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass