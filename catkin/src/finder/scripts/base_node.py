#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

"""
El proposito de esta clase es servir de nodo de control de bajo nivel para el motor de la base del robot, el microcontrolador
publica un topico de tipo Int16 llamada "base_lec" y consume otro topico de tipo Int16 llamado "base_out". base_lec es la
lectura sin procesar del encoder, y base_out es el valor de salida PWM del motor, que puede ser un valor de -100 a 100. El
procesamiento de los datos del encoder no es tan trivial, pero se incluyen las funciones necesarias para realizarlo.

Esta clase debe suscrivirse a un topico Int16 llamado "base_des" y publicar dos topicos Int16 llamados "base_ang" y 
"base_vel" que contengan informacion del angulo y la velocidad. Internamente, el control se realiza sobre el valor del
angulo, haciendo un match PID entre base_des y base_ang.

ESTE MOTOR SE CONTROLA A BAJO NIVEL POR POSICION
"""

class Base_node:

    def __init__(self, node_name_override = 'base_node'):
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("base_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        self.baseLecSub = rospy.Subscriber("base_lec", Int16, self.baseLecCb)
        self.baseDesSub = rospy.Subscriber("base_des", Float32, self.baseDesCb)
        self.baseOutPub = rospy.Publisher("base_out", Int16)
        self.baseAngPub = rospy.Publisher("base_ang", Float32)
        self.baseVelPub = rospy.Publisher("base_vel", Float32)

        #TODO cambiar por parametros
        self.base_enc_ana = False
        self.base_enc_max = 1023
        #self.base_enc_lim = 100
        #self.base_lec_min = 485
        #self.base_lec_max = 1004
        #self.base_lec_dir = -100
        self.base_ang_def = 0
        self.base_offset = 745

        # PID control parameters
        self.kp = 100
        self.ki = 10
        self.kd = 0
        self.km = 0
        self.umbral = 0.1
        self.range = 100 # Maximo pwm permitido
        self.kierr = 1.2
        self.kimax = 100
        self.kisum = 0
        self.error = 0

        # topic variables
        self.base_lec = 0
        self.base_ang = 0
        self.base_des = 0
        self.base_vel = 0
        self.base_out = 0

        # helper variables
        self.base_ang_tmp = 0
        self.base_ang_rng = 0
        self.base_lec_dst = 0
        self.base_ang_lst = 0
        self.base_ang_chg = 0

        self.base_ang_abs = 0
        self.lap = 0

        # inits
        self.init_time = rospy.get_time()
        #self.angInit()


    def map(self, x, in_min, in_max, out_min, out_max):

        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


    def millis(self):

        return int(1000 * (self.rospy.get_time() - self.init_time))


    def constrain(self, x, min, max):

        if (x > max):
            return max
        if (x < min):
            return min
        return x


    def pid(self):

        if (abs(self.base_des - self.base_ang) <= self.umbral):
           self.error = 0.
        else:
            self.error = self.base_des - self.base_ang + 0.
            #print "error " + str(self.error)


        if (abs(self.base_des - self.base_ang) < self.kierr):
            if (abs(self.base_des - self.base_ang) < self.umbral):
                self.kisum = 0.;
            else:
                self.kisum += self.ki * self.error
                self.kisum = self.constrain(self.kisum, -self.kimax, self.kimax)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum = 0.

        self.base_out = self.constrain(self.kp * self.error + self.kisum - self.kd * (self.base_ang - self.base_ang_lst) + self.km * self.base_des, -self.range, self.range)


    """
    def angInit(self):

        # Calculamos en rango de lecturas del encoder, que depender de la direccion en que se mueven
        # los valores del encoder
        if (self.base_lec_dir > 0):
            self.base_lec_rng = self.base_lec_max - self.base_lec_min
        else:
            self.base_lec_rng = self.base_lec_min - self.base_lec_max

        # Quiza haya que corregir modulo el valor maximo del encoder
        if (self.base_lec_rng < 0):
            self.base_lec_rng += self.base_enc_max
    """


    def angCalc(self):

        # # Calculamos la "distancia" del encoder a su minimo
        # if (slef.base_lec_dir > 0):
        # 	self.base_lec_dst = self.base_lec - self.base_lec_min
        # else:
        # 	self.base_lec_dst = self.base_lec_min - self.base_lec

        # # y las siguientes correcciones necesitan arreglos
        # if (self.base_lec_dst > 0 and self.base_lec_dir > 0):
        # 	self.base_lec_dst += self.base_enc_max

        # if (self.base_lec_dst < 0 and self.base_lec_dir < 0):
        # 	if (self.base_lec_dst < -self.base_enc_max / 3):
        # 		self.base_lec_dst += self.base_enc_max

        # self.base_ang_tmp = self.map(self.base_lec_dst, 0, self.base_lec_rng, -abs(self.base_lec_dir), abs(self.base_lec_dir))

        # # ESTE ENCODER NO ES ANALOGICO

        # if (self.base_enc_ana):
        # 	if (abs(self.base_ang_tmp - self.base_ang) > self.base_enc_lim):
        # 		if (self.millis() > 10000):
        # 			return 0

        # self.base_ang_lst = self.base_ang
        # self.base_ang = self.base_ang_tmp

        # self.base_ang_chg = self.base_ang - self.base_ang_lst

        # if (self.base_ang_chg > abs(self.base_lec_dir)):
        # 	self.base_ang_chg -= 2 * abs(self.base_lec_dir) + 1
        # if (self.base_ang_chg < -abs(self.base_lec_dir)):
        # 	self.base_ang_chg += 2 * abs(self.base_lec_dir) + 1

        # self.base_vel = selg.base_ang_chg

        """MAP FIRST"""
        if self.base_lec < self.base_offset:
            self.base_ang_tmp = self.base_lec + 1024 - self.base_offset
        else:
            self.base_ang_tmp = self.base_lec - self.base_offset

        self.base_ang_tmp = self.map(self.base_ang_tmp, 0., 1023., 2*pi, 0.)
        self.base_ang_lst = self.base_ang_abs
        self.base_ang_abs = self.base_ang_tmp
        """MAP FIRST"""

        """LAP CALCULATE"""
        # encuentra si el cambio fue de 0 a 2pi
        if (self.base_ang_abs > 1.8 * pi and self.base_ang_lst < 0.2 * pi):
            self.lap -= 1
        # encuetra si el cambio due de 2pi a 0
        if (self.base_ang_abs < 0.2 * pi and self.base_ang_lst > 1.8 * pi):
            self.lap += 1
        """LAP CALCULATE"""

        self.base_ang_lap_lst = self.base_ang
        self.base_ang = 2 * pi * self.lap + self.base_ang_abs

        """MAP VEL OUT"""
        self.base_vel = self.base_ang - self.base_ang_lap_lst
        """MAP VEL OUT"""

    def desCalc(self, data):

        self.base_des = self.constrain(self.base_des, )


    def baseLecCb(self, data):
        self.base_lec = data.data
        self.angCalc()
        self.pid()


    def baseDesCb(self, data):
        self.base_des = data.data
        self.angCalc()
        self.pid()


    def update(self):
        self.baseOutPub.publish(self.base_out)
        self.baseAngPub.publish(self.base_ang)
        self.baseVelPub.publish(self.base_vel)


    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':
    """ main """
    base_node = Base_node()
    base_node.spin()
    
