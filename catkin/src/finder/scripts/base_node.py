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
"base_vel" que contengan informacion del angulo y la velocidad. Internamente, el control se realiza sobasee el valor del
angulo, haciendo un match PID entre base_des y base_ang.

ESTE MOTOR SE CONTROLA A BAJO NIVEL POR POSICION
"""

class Fl_node:

    def __init__(self, node_name_override = 'base_node'):
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("base_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        self.baseOutPub = rospy.Publisher("base_out", Int16)
        self.baseAngPub = rospy.Publisher("base_ang", Float32)
        self.baseVelPub = rospy.Publisher("base_vel", Float32)

        #TODO cambiar por parametros
        self.base_enc_ana = False
        self.base_enc_max = 1023
        self.base_offset = 245
        #self.base_enc_lim = 100
        #self.base_lec_min = 485
        #self.base_lec_max = 1004
        #self.base_lec_dir = -100

        # PID control parameters
        self.kp_pos = 50
        self.ki_pos = 5.
        self.kd_pos = 0.
        self.km_pos = 0.
        self.umbaseal_pos = 0.1
        self.range_pos = 50. # Maximo pwm permitido
        self.kierr_pos = 2
        self.kimax_pos = 25.
        self.kisum_pos = 0.
        self.error_pos = 0.

        self.kp_vel = 50
        self.ki_vel = 5.
        self.kd_vel = 0.
        self.km_vel = 0.
        self.umbaseal_vel = 0.1
        self.range_vel = 50. # Maximo pwm permitido
        self.kierr_vel = 2
        self.kimax_vel = 25.
        self.kisum_vel = 0.
        self.error_vel = 0.

        # topic variables
        self.base_lec = 0
        #self.base_des = 0
        self.base_ang = 0
        self.base_ang_des = 0   
        self.base_vel = 0
        self.base_vel_des = 0
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

        self.init_baseag = False;

        self.baseResetSub = rospy.Subscriber("base_reset", Int16, self.baseResetCb)
        self.baseLecSub = rospy.Subscriber("base_lec", Int16, self.baseLecCb)
        self.baseDesSub = rospy.Subscriber("base_des", Float32, self.baseDesCb)
        self.offsetSub = rospy.Subscriber("offset", Int16, self.offsetCb)

    def offsetCb(self, data):

        if(data.data == 1):

            self.base_offset  = self.base_lec
            
            self.base_ang_tmp = 0
            self.base_ang_lst = 0
            self.base_ang_abs = 0

            self.base_ang = 0
            self.base_ang_lap =  0
            self.base_ang_lap_lst = 0


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

        if (abs(self.base_ang_des - self.base_ang) <= self.umbaseal_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.base_ang_des - self.base_ang
            #print "error " + str(self.error)


        if (abs(self.base_ang_des - self.base_ang) < self.kierr_pos):
            if (abs(self.base_ang_des - self.base_ang) < self.umbaseal_pos):
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_pos = 0.

        self.base_out = self.constrain(self.kp_pos * self.error_pos + self.kisum_pos + self.km_pos * self.base_ang_des, -self.range_pos, self.range_pos)

    def pid_vel(self):

        if (abs(self.base_vel_des - self.base_vel) <= self.umbaseal_vel):
           self.error_vel = 0.
        else:
            self.error_vel = self.base_vel_des - self.base_vel + 0.
            #print "error " + str(self.error)


        if (abs(self.base_vel_des - self.base_vel) < self.kierr_vel):
            if (abs(self.base_vel_des - self.base_vel) < self.umbaseal_vel):
                self.kisum_vel = 0.;
            else:
                self.kisum_vel += self.ki_vel * self.error_vel
                self.kisum_vel = self.constrain(self.kisum_vel, -self.kimax_vel, self.kimax_vel)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_vel = 0.

        self.base_out = self.constrain(self.kp_vel * self.error_vel + self.kisum_vel + self.km_vel * self.base_vel_des, -self.range_vel, self.range_vel)


    def angCalc(self):

        """MAP FIRST"""
        """
        if self.base_lec < self.base_offset:
            self.base_ang_tmp = self.base_lec + 1024 - self.base_offset
        else:
            self.base_ang_tmp = self.base_lec - self.base_offset
        """
        
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
        
        self.base_ang_lap_lst = self.base_ang
        self.base_ang = 2 * pi * self.lap + self.base_ang_abs
        """LAP CALCULATE"""

        """MAP VEL OUT"""
        self.base_vel = 10. * (self.base_ang - self.base_ang_lap_lst)
        """MAP VEL OUT"""

    def baseResetCb(self, data):
        if (data.data == 1):
            self.base_ang_des = 0

    def baseLecCb(self, data):
        self.base_lec = data.data
        self.angCalc()

        if (abs(self.base_vel_des) < 0.2):
            # self.base_ang_des = self.constrain(self.base_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.base_ang_des)
        else:
            self.base_ang_des = self.base_ang
            self.pid_vel()


    def baseDesCb(self, data):
        self.base_vel_des = data.data
        self.angCalc()

        if (abs(self.base_vel_des) < 0.1):
            # self.base_ang_des = self.constrain(self.base_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.base_ang_des)
        else:
            self.base_ang_des = self.base_ang
            self.pid_vel()


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
    base_node = Fl_node()
    base_node.spin()
    
