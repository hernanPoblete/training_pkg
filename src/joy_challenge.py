#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 #importa mensajes de ROS tipo String y Int32
from sensor_msgs.msg import Joy # impor mensaje tipo Joy
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from duckietown_msgs.msg import Twist2DStamped 
from math import *

class Template(object):
    def __init__(self, args):
        super(Template, self).__init__()
        self.args = args
        #sucribir a joy 
        self.sub = rospy.Subscriber("/duckiebot/joy" , Joy, self.callback)
        #publicar la intrucciones del control en possible_cmd
        self.publi = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped)
        self.twist = Twist2DStamped()


    #def publicar(self, msg):
        #self.publi.publish(msg)

    def callback(self,msg):
        tol = 0.07
        r2 = (msg.axes[5]-1)
        l2 = -(msg.axes[2]-1)
        v = 0
        if r2 != 0:
            v =r2
        else:
            v = l2
        
        y = 0
        if not tol > abs(msg.axes[1]):
            y=msg.axes[1]
        x = 0
        if not tol > abs(msg.axes[0]):
            x = msg.axes[0]
        theta = 0
        if x!=0:
            theta = atan(y/x)
        
        print(y, x)
        self.twist.omega = -10*x
        self.twist.v =  v
        self.publi.publish(self.twist)
        




def main():
    rospy.init_node('test') #creacion y registro del nodo!

    obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

    #objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

    rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
    main()
