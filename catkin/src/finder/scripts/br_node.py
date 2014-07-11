#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

"""
El proposito de esta clase es servir de nodo de control de bajo nivel para el motor de la br del robot, el microcontrolador
publica un topico de tipo Int16 llamada "br_lec" y consume otro topico de tipo Int16 llamado "br_out". br_lec es la
lectura sin procesar del encoder, y br_out es el valor de salida PWM del motor, que puede ser un valor de -100 a 100. El
procesamiento de los datos del encoder no es tan trivial, pero se incluyen las funciones necesarias para realizarlo.

Esta clase debe suscrivirse a un topico Int16 llamado "br_des" y publicar dos topicos Int16 llamados "br_ang" y 
"br_vel" que contengan informacion del angulo y la velocidad. Internamente, el control se realiza sobre el valor del
angulo, haciendo un match PID entre br_des y br_ang.

ESTE MOTOR SE CONTROLA A BAJO NIVEL POR POSICION
"""

class Br_node:

    def __init__(self, node_name_override = 'br_node'):
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("br_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        self.brOutPub = rospy.Publisher("br_out", Int16)
        self.brAngPub = rospy.Publisher("br_ang", Float32)
        self.brVelPub = rospy.Publisher("br_vel", Float32)

        #TODO cambiar por parametros
        self.br_enc_ana = False
        self.br_enc_max = 1023
        self.br_offset = 185
        #self.br_enc_lim = 100
        #self.br_lec_min = 485
        #self.br_lec_max = 1004
        #self.br_lec_dir = -100

        # PID control parameters
        self.kp_pos = 20
        self.ki_pos = 5
        self.kd_pos = 0
        self.km_pos = 0
        self.umbral_pos = 0.5
        self.range_pos = 50 # Maximo pwm permitido
        self.kierr_pos = 5
        self.kimax_pos = 50
        self.kisum_pos = 0
        self.error_pos = 0

        self.kp_vel = 5
        self.ki_vel = 2
        self.kd_vel = 0
        self.km_vel = 0
        self.umbral_vel = 0.5
        self.range_vel = 50 # Maximo pwm permitido
        self.kierr_vel = 1
        self.kimax_vel = 50
        self.kisum_vel = 0
        self.error_vel = 0

        # topic variables
        self.br_lec = 0
        #self.br_des = 0
        self.br_ang = 0
        self.br_ang_des = 0   
        self.br_vel = 0
        self.br_vel_des = 0
        self.br_out = 0

        # helper variables
        self.br_ang_tmp = 0
        self.br_ang_rng = 0
        self.br_lec_dst = 0
        self.br_ang_lst = 0
        self.br_ang_chg = 0

        self.br_ang_abs = 0
        self.lap = 0

        # inits
        self.init_time = rospy.get_time()
        #self.angInit()

        self.br_offset_internal = 0.
        self.br_ang_tmp_internal = 0      
        self.br_ang_lst_internal = 0
        self.br_ang_abs_internal = 0

        self.times = 0        

        self.init_flag = False;

        self.brResetSub = rospy.Subscriber("br_reset", Int16, self.brResetCb)
        self.brLecSub = rospy.Subscriber("br_lec", Int16, self.brLecCb)
        self.brDesSub = rospy.Subscriber("br_des", Float32, self.brDesCb)
        self.offsetSub = rospy.Subscriber("offset", Int16, self.offsetCb)

    def offsetCb(self, data):

        if (data.data == 1):

            self.br_offset  = self.br_lec
            
            self.br_ang_tmp = 0
            self.br_ang_lst = 0
            self.br_ang_abs = 0

            self.br_ang = 0
            self.br_ang_lap =  0
            self.br_ang_lap_lst = 0


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


    def pid_pos(self):

        if (abs(self.br_ang_des - self.br_ang) <= self.umbral_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.br_ang_des - self.br_ang
            #print "error " + str(self.error)


        if (abs(self.br_ang_des - self.br_ang) < self.kierr_pos):
            if (abs(self.br_ang_des - self.br_ang) < self.umbral_pos):
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_pos = 0.

        self.br_out = self.constrain(self.kp_pos * self.error_pos + self.kisum_pos + self.km_pos * self.br_ang_des, -self.range_pos, self.range_pos)

    def pid_vel(self):

        if (abs(self.br_vel_des - self.br_vel) <= self.umbral_vel):
           self.error_vel = 0.
        else:
            self.error_vel = self.br_vel_des - self.br_vel + 0.
            #print "error " + str(self.error)


        if (abs(self.br_vel_des - self.br_vel) < self.kierr_vel):
            if (abs(self.br_vel_des - self.br_vel) < self.umbral_vel):
                self.kisum_vel = 0.;
            else:
                self.kisum_vel += self.ki_vel * self.error_vel
                self.kisum_vel = self.constrain(self.kisum_vel, -self.kimax_vel, self.kimax_vel)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_vel = 0.

        self.br_out = self.constrain(self.kp_vel * self.error_vel + self.kisum_vel + self.km_vel * self.br_vel_des, -self.range_vel, self.range_vel)


    def angCalc(self):

        """MAP FIRST"""
        """
        if self.br_lec < self.br_offset:
            self.br_ang_tmp = self.br_lec + 1024 - self.br_offset
        else:
            self.br_ang_tmp = self.br_lec - self.br_offset
        """
        self.br_ang_tmp_internal = self.map(self.br_lec, 0., self.br_enc_max, 0, 2 * pi)
        self.br_ang_lst_internal = self.br_ang_abs_internal
        self.br_ang_abs_internal = self.br_ang_tmp_internal
        if (self.times > 4):
            # encuentra si el cambio fue de 0 a 2pi
            if (self.br_ang_abs_internal > 1.7 * pi and self.br_ang_lst_internal < 0.3 * pi):
                self.br_ang_lap -= 1
            # encuentra si el cambio fue de 2pi a 0
            if (self.br_ang_abs_internal < 0.3 * pi and self.br_ang_lst_internal > 1.7 * pi):
                self.br_ang_lap += 1    
        """
        self.br_ang_tmp = self.br_lec        
        self.br_ang_tmp = self.map(self.br_ang_tmp, 0., 1023., 2*pi, 0.)
        self.br_ang_lst = self.br_ang_abs
        self.br_ang_abs = self.br_ang_tmp
        """
        """MAP FIRST"""

        """LAP CALCULATE"""
        """
        # encuentra si el cambio fue de 0 a 2pi
        if (self.br_ang_abs > 1.8 * pi and self.br_ang_lst < 0.2 * pi):
            self.lap -= 1
        # encuetra si el cambio due de 2pi a 0
        if (self.br_ang_abs < 0.2 * pi and self.br_ang_lst > 1.8 * pi):
            self.lap += 1
        """
        self.br_ang_lap_lst = self.br_ang
        self.br_ang = 2 * pi * self.lap + self.br_ang_abs
        """LAP CALCULATE"""

        """MAP VEL OUT"""
        self.br_vel = 10. * (self.br_ang - self.br_ang_lap_lst)
        """MAP VEL OUT"""

    def brResetCb(self, data):
        if (data.data == 1):
            self.br_ang_des = 0

    def brLecCb(self, data):
        self.br_lec = data.data
        self.angCalc()

        if (abs(self.br_vel_des) < 0.2):
            # self.br_ang_des = self.constrain(self.br_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.br_ang_des)
        else:
            self.br_ang_des = self.br_ang
            self.pid_vel()


    def brDesCb(self, data):
        self.br_vel_des = data.data
        self.angCalc()

        if (abs(self.br_vel_des) < 0.2):
            # self.br_ang_des = self.constrain(self.br_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.br_ang_des)
        else:
            self.br_ang_des = self.br_ang
            self.pid_vel()


    def update(self):
        self.brOutPub.publish(self.br_out)
        self.brAngPub.publish(self.br_ang)
        self.brVelPub.publish(self.br_vel)


    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':
    """ main """
    br_node = Br_node()
    br_node.spin()
    
