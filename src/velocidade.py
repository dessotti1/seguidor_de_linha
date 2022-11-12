#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist

class Velocidade:

    def __init__(self):

        self.publish_time = 0.01

        self.linear = 0
        self.angular = 0
        self.counter = 0

        self.dados = [-1, -1, -1, -1, -1]
        extEsq = self.dados[0]
        esq = self.dados[1]
        center = self.dados[2]
        dir = self.dados[3]
        extDir = self.dados[4]

        rospy.Subscriber("/dados", Int32MultiArray, self.mapeamento, queue_size = 10)

        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.timer_pub = rospy.Timer(rospy.Duration(self.publish_time), self.timerCallback)  


    def mapeamento (self, msg):
        self.dados = msg.data

        extEsq = self.dados[0]
        esq = self.dados[1]
        center = self.dados[2]
        dir = self.dados[3]
        extDir = self.dados[4]

        if (extEsq == 1 and esq == 1 and center == 0 and dir == 1 and extDir == 1):
            self.linear = 0.3
            self.angular = 0
            print("Andando\n")

        elif (extEsq == 0 and esq == 0 and center == 0 and dir == 0 and extDir == 0):
            self.linear = 0.3
            self.counter += 1
            print("Cruzamento detectado.\n")


        elif (extEsq == 0 and esq == 0 and center == 1 and dir == 1 and extDir == 1):
            self.angular = 0.8
            self.linear = 0.12
            print("Curva à esquerda detectada.\n")

        elif (extEsq == 1 and esq == 0 and center == 1 and dir == 1 and extDir == 1):
            self.angular = 0.8
            self.linear = 0.12
            print("Curva à esquerda detectada.\n")


        elif (extEsq == 0 and esq == 0 and center == 0 and dir == 1 and extDir == 1):
            self.angular = 0.5
            self.linear = 0.12
            print("Curva à esquerda detectada.\n")

        
        elif (extEsq == 1 and esq == 1 and center == 1 and dir == 0 and extDir == 0):
            self.angular = -0.8
            self.linear = 0.12
            print("Curva à direita detectada.\n")

        
        elif (extEsq == 1 and esq == 1 and center == 1 and dir == 0 and extDir == 1):
            self.angular = -0.8
            self.linear = 0.12
            print("Curva à direita detectada.\n")
        
        
        elif (extEsq == 1 and esq == 1 and center == 0 and dir == 0 and extDir == 1):
            self.linear = 0.1

        
        elif (extEsq == 1 and esq == 0 and center == 0 and dir == 1 and extDir == 1):
            self.linear = 0.1

        elif (extEsq == 1 and esq == 0 and center == 0 and dir == 0 and extDir == 1):
            self.linear = 0.05
        

    def timerCallback(self, event):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.cmd_vel.publish(msg)
        



if __name__ == '__main__':

    try:
        rospy.init_node("velocidade")
        Velocidade()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass  