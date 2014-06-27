#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

"""
El proposito de esta clase es servir de nodo de control de bajo nivel para el motor de la fl del robot, el microcontrolador
publica un topico de tipo Int16 llamada "fl_lec" y consume otro topico de tipo Int16 llamado "fl_out". fl_lec es la
lectura sin procesar del encoder, y fl_out es el valor de salida PWM del motor, que puede ser un valor de -100 a 100. El
procesamiento de los datos del encoder no es tan trivial, pero se incluyen las funciones necesarias para realizarlo.

Esta clase debe suscrivirse a un topico Int16 llamado "fl_des" y publicar dos topicos Int16 llamados "fl_ang" y 
"fl_vel" que contengan informacion del angulo y la velocidad. Internamente, el control se realiza sofle el valor del
angulo, haciendo un match PID entre fl_des y fl_ang.

ESTE MOTOR SE CONTROLA A BAJO NIVEL POR POSICION
"""

class Fl_node:

    def __init__(self, node_name_override = 'fl_node'):
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("fl_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        self.flOutPub = rospy.Publisher("fl_out", Int16)
        self.flAngPub = rospy.Publisher("fl_ang", Float32)
        self.flVelPub = rospy.Publisher("fl_vel", Float32)

        #TODO cambiar por parametros
        self.fl_enc_ana = False
        self.fl_enc_max = 1023
        self.fl_offset = 766
        #self.fl_enc_lim = 100
        #self.fl_lec_min = 485
        #self.fl_lec_max = 1004
        #self.fl_lec_dir = -100

        # PID control parameters
        self.kp_pos = 7
        self.ki_pos = 1
        self.kd_pos = 0
        self.km_pos = 0
        self.umflal_pos = 1
        self.range_pos = 50 # Maximo pwm permitido
        self.kierr_pos = 2
        self.kimax_pos = 50
        self.kisum_pos = 0
        self.error_pos = 0

        self.kp_vel = 5
        self.ki_vel = 2
        self.kd_vel = 0
        self.km_vel = 0
        self.umflal_vel = 0.5
        self.range_vel = 50 # Maximo pwm permitido
        self.kierr_vel = 1
        self.kimax_vel = 50
        self.kisum_vel = 0
        self.error_vel = 0

        # topic variables
        self.fl_lec = 0
        #self.fl_des = 0
        self.fl_ang = 0
        self.fl_ang_des = 0   
        self.fl_vel = 0
        self.fl_vel_des = 0
        self.fl_out = 0

        # helper variables
        self.fl_ang_tmp = 0
        self.fl_ang_rng = 0
        self.fl_lec_dst = 0
        self.fl_ang_lst = 0
        self.fl_ang_chg = 0

        self.fl_ang_abs = 0
        self.lap = 0

        # inits
        self.init_time = rospy.get_time()
        #self.angInit()

        self.init_flag = False;

        self.flResetSub = rospy.Subscriber("fl_reset", Int16, self.flResetCb)
        self.flLecSub = rospy.Subscriber("fl_lec", Int16, self.flLecCb)
        self.flDesSub = rospy.Subscriber("fl_des", Float32, self.flDesCb)
        self.offsetSub = rospy.Subscriber("offset", Int16, self.offsetCb)

    def offsetCb(self):

        self.fl_offset  = self.base_lec
        
        self.fl_ang_tmp = 0
        self.fl_ang_lst = 0
        self.fl_ang_abs = 0

        self.fl_ang = 0
        self.fl_ang_lap =  0
        self.fl_ang_lap_lst = 0


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

        if (abs(self.fl_ang_des - self.fl_ang) <= self.umflal_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.fl_ang_des - self.fl_ang
            #print "error " + str(self.error)


        if (abs(self.fl_ang_des - self.fl_ang) < self.kierr_pos):
            if (abs(self.fl_ang_des - self.fl_ang) < self.umflal_pos):
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_pos = 0.

        self.fl_out = self.constrain(self.kp_pos * self.error_pos + self.kisum_pos + self.km_pos * self.fl_ang_des, -self.range_pos, self.range_pos)

    def pid_vel(self):

        if (abs(self.fl_vel_des - self.fl_vel) <= self.umflal_vel):
           self.error_vel = 0.
        else:
            self.error_vel = self.fl_vel_des - self.fl_vel + 0.
            #print "error " + str(self.error)


        if (abs(self.fl_vel_des - self.fl_vel) < self.kierr_vel):
            if (abs(self.fl_vel_des - self.fl_vel) < self.umflal_vel):
                self.kisum_vel = 0.;
            else:
                self.kisum_vel += self.ki_vel * self.error_vel
                self.kisum_vel = self.constrain(self.kisum_vel, -self.kimax_vel, self.kimax_vel)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_vel = 0.

        self.fl_out = self.constrain(self.kp_vel * self.error_vel + self.kisum_vel + self.km_vel * self.fl_vel_des, -self.range_vel, self.range_vel)


    def angCalc(self):

        """MAP FIRST"""
        if self.fl_lec < self.fl_offset:
            self.fl_ang_tmp = self.fl_lec + 1024 - self.fl_offset
        else:
            self.fl_ang_tmp = self.fl_lec - self.fl_offset

        self.fl_ang_tmp = self.map(self.fl_ang_tmp, 0., 1023., 2*pi, 0.)
        self.fl_ang_lst = self.fl_ang_abs
        self.fl_ang_abs = self.fl_ang_tmp
        """MAP FIRST"""

        """LAP CALCULATE"""
        # encuentra si el cambio fue de 0 a 2pi
        if (self.fl_ang_abs > 1.5 * pi and self.fl_ang_lst < 0.5 * pi):
            self.lap -= 1
        # encuetra si el cambio due de 2pi a 0
        if (self.fl_ang_abs < 0.5 * pi and self.fl_ang_lst > 1.5 * pi):
            self.lap += 1
        
        self.fl_ang_lap_lst = self.fl_ang
        self.fl_ang = 2 * pi * self.lap + self.fl_ang_abs
        """LAP CALCULATE"""

        """MAP VEL OUT"""
        self.fl_vel = 10. * (self.fl_ang - self.fl_ang_lap_lst)
        """MAP VEL OUT"""

    def flResetCb(self, data):
        if (data.data == 1):
            self.fl_ang_des = 0

    def flLecCb(self, data):
        self.fl_lec = data.data
        self.angCalc()

        if (abs(self.fl_vel_des) < 0.2):
            # self.fl_ang_des = self.constrain(self.fl_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.fl_ang_des)
        else:
            self.fl_ang_des = self.fl_ang
            self.pid_vel()


    def flDesCb(self, data):
        self.fl_vel_des = data.data
        self.angCalc()

        if (abs(self.fl_vel_des) < 0.2):
            # self.fl_ang_des = self.constrain(self.fl_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.fl_ang_des)
        else:
            self.fl_ang_des = self.fl_ang
            self.pid_vel()


    def update(self):
        self.flOutPub.publish(self.fl_out)
        self.flAngPub.publish(self.fl_ang)
        self.flVelPub.publish(self.fl_vel)


    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':
    """ main """
    fl_node = Fl_node()
    fl_node.spin()
    
