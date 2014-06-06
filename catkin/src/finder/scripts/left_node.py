#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

"""
El proposito de esta clase es servir de nodo de control de bajo nivel para el motor de la left del robot, el microcontrolador
publica un topico de tipo Int16 llamada "left_lec" y consume otro topico de tipo Int16 llamado "left_out". left_lec es la
lectura sin procesar del encoder, y left_out es el valor de salida PWM del motor, que puede ser un valor de -100 a 100. El
procesamiento de los datos del encoder no es tan trivial, pero se incluyen las funciones necesarias para realizarlo.

Esta clase debe suscrivirse a un topico Int16 llamado "left_des" y publicar dos topicos Int16 llamados "left_ang" y 
"left_vel" que contengan informacion del angulo y la velocidad. Internamente, el control se realiza sobre el valor del
angulo, haciendo un match PID entre left_des y left_ang.

ESTE MOTOR SE CONTROLA A BAJO NIVEL POR POSICION
"""

class Left_node:

    def __init__(self, node_name_override = 'left_node'):
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("left_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        self.leftOutPub = rospy.Publisher("left_out", Int16)
        self.leftAngPub = rospy.Publisher("left_ang", Float32)
        self.leftVelPub = rospy.Publisher("left_vel", Float32)

        #TODO cambiar por parametros
        self.left_enc_ana = False
        self.left_enc_max = 1023
        #self.left_enc_lim = 100
        #self.left_lec_min = 485
        #self.left_lec_max = 1004
        #self.left_lec_dir = -100
        self.left_ang_def = 0
        self.left_offset = 0

        # PID control parameters
        self.kp = 10
        self.ki = 0
        self.kd = 0
        self.km = 10
        self.umbral = 0.1
        self.range = 100 # Maximo pwm permitido
        self.kierr = 1.2
        self.kimax = 100
        self.kisum = 0
        self.error = 0

        # topic variables
        self.left_lec = 0
        self.left_ang = 0
        self.left_des = 0
        self.left_vel = 0
        self.left_out = 0

        # helper variables
        self.left_ang_tmp = 0
        self.left_ang_rng = 0
        self.left_lec_dst = 0
        self.left_ang_lst = 0
        self.left_ang_chg = 0

        self.left_ang_abs = 0
        self.lap = 0

        # inits
        self.init_time = rospy.get_time()
        #self.angInit()

        self.leftLecSub = rospy.Subscriber("left_lec", Int16, self.leftLecCb)
        self.leftDesSub = rospy.Subscriber("left_des", Float32, self.leftDesCb)


    def map(self, x, in_min, in_max, out_min, out_max):

        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


    def millis(self):

        return 1000 * (rospy.get_time() - self.init_time)


    def constrain(self, x, min, max):

        if (x > max):
            return max
        if (x < min):
            return min
        return x


    def pid(self):

        if (abs(self.left_des - self.left_vel) <= self.umbral):
           self.error = 0.
        else:
            self.error = self.left_des - self.left_vel + 0.
            #print "error " + str(self.error)


        if (abs(self.left_des - self.left_vel) < self.kierr):
            if (abs(self.left_des - self.left_vel) < self.umbral):
                self.kisum = 0.;
            else:
                self.kisum += self.ki * self.error
                self.kisum = self.constrain(self.kisum, -self.kimax, self.kimax)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum = 0.

        self.left_out = self.constrain(self.kp * self.error + self.kisum + self.km * self.left_des, -self.range, self.range)


    def angCalc(self):

        """MAP FIRST"""
        if self.left_lec < self.left_offset:
            self.left_ang_tmp = self.left_lec + 1024 - self.left_offset
        else:
            self.left_ang_tmp = self.left_lec - self.left_offset

        self.left_ang_tmp = self.map(self.left_ang_tmp, 0., 1023., 2*pi, 0.)
        self.left_ang_lst = self.left_ang_abs
        self.left_ang_abs = self.left_ang_tmp
        """MAP FIRST"""

        """LAP CALCULATE"""
        # encuentra si el cambio fue de 0 a 2pi
        if (self.left_ang_abs > 1.5 * pi and self.left_ang_lst < 0.5 * pi):
            self.lap -= 1
        # encuetra si el cambio due de 2pi a 0
        if (self.left_ang_abs < 0.5 * pi and self.left_ang_lst > 1.5 * pi):
            self.lap += 1
        
        self.left_ang_lap_lst = self.left_ang
        self.left_ang = 2 * pi * self.lap + self.left_ang_abs
        """LAP CALCULATE"""

        """MAP VEL OUT"""
        self.left_vel = 10. * (self.left_ang - self.left_ang_lap_lst)
        """MAP VEL OUT"""

    def leftLecCb(self, data):
        self.left_lec = data.data
        self.angCalc()
        self.pid()


    def leftDesCb(self, data):
        self.left_des = self.constrain(data.data, -80, 80)
        self.angCalc()
        self.pid()


    def update(self):
        self.leftOutPub.publish(self.left_des)
        self.leftAngPub.publish(self.left_ang)
        self.leftVelPub.publish(self.left_vel)


    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':
    """ main """
    left_node = Left_node()
    left_node.spin()
    
