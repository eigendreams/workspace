#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

"""
El proposito de esta clase es servir de nodo de control de bajo nivel para el motor de la bl del robot, el microcontrolador
publica un topico de tipo Int16 llamada "bl_lec" y consume otro topico de tipo Int16 llamado "bl_out". bl_lec es la
lectura sin procesar del encoder, y bl_out es el valor de salida PWM del motor, que puede ser un valor de -100 a 100. El
procesamiento de los datos del encoder no es tan trivial, pero se incluyen las funciones necesarias para realizarlo.

Esta clase debe suscrivirse a un topico Int16 llamado "bl_des" y publicar dos topicos Int16 llamados "bl_ang" y 
"bl_vel" que contengan informacion del angulo y la velocidad. Internamente, el control se realiza soble el valor del
angulo, haciendo un match PID entre bl_des y bl_ang.

ESTE MOTOR SE CONTROLA A BAJO NIVEL POR POSICION
"""

class Bl_node:

    def __init__(self, node_name_override = 'bl_node'):
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("bl_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        self.blOutPub = rospy.Publisher("bl_out", Int16)
        self.blAngPub = rospy.Publisher("bl_ang", Float32)
        self.blVelPub = rospy.Publisher("bl_vel", Float32)

        #TODO cambiar por parametros
        self.bl_enc_ana = False
        self.bl_enc_max = 1023
        self.bl_offset = 751
        #self.bl_enc_lim = 100
        #self.bl_lec_min = 485
        #self.bl_lec_max = 1004
        #self.bl_lec_dir = -100

        # PID control parameters
        self.kp_pos = 20
        self.ki_pos = 5
        self.kd_pos = 0
        self.km_pos = 0
        self.umblal_pos = 0.5
        self.range_pos = 50 # Maximo pwm permitido
        self.kierr_pos = 5
        self.kimax_pos = 50
        self.kisum_pos = 0
        self.error_pos = 0

        self.kp_vel = 5
        self.ki_vel = 2
        self.kd_vel = 0
        self.km_vel = 0
        self.umblal_vel = 0.5
        self.range_vel = 50 # Maximo pwm permitido
        self.kierr_vel = 1
        self.kimax_vel = 50
        self.kisum_vel = 0
        self.error_vel = 0

        # topic variables
        self.bl_lec = 0
        #self.bl_des = 0
        self.bl_ang = 0
        self.bl_ang_des = 0   
        self.bl_vel = 0
        self.bl_vel_des = 0
        self.bl_out = 0

        # helper variables
        self.bl_ang_tmp = 0
        self.bl_ang_rng = 0
        self.bl_lec_dst = 0
        self.bl_ang_lst = 0
        self.bl_ang_chg = 0

        self.bl_ang_abs = 0
        self.lap = 0

        # inits
        self.init_time = rospy.get_time()
        #self.angInit()

        self.init_flag = False;

        self.blResetSub = rospy.Subscriber("bl_reset", Int16, self.blResetCb)
        self.blLecSub = rospy.Subscriber("bl_lec", Int16, self.blLecCb)
        self.blDesSub = rospy.Subscriber("bl_des", Float32, self.blDesCb)


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

        if (abs(self.bl_ang_des - self.bl_ang) <= self.umblal_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.bl_ang_des - self.bl_ang
            #print "error " + str(self.error)


        if (abs(self.bl_ang_des - self.bl_ang) < self.kierr_pos):
            if (abs(self.bl_ang_des - self.bl_ang) < self.umblal_pos):
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_pos = 0.

        self.bl_out = self.constrain(self.kp_pos * self.error_pos + self.kisum_pos + self.km_pos * self.bl_ang_des, -self.range_pos, self.range_pos)

    def pid_vel(self):

        if (abs(self.bl_vel_des - self.bl_vel) <= self.umblal_vel):
           self.error_vel = 0.
        else:
            self.error_vel = self.bl_vel_des - self.bl_vel + 0.
            #print "error " + str(self.error)


        if (abs(self.bl_vel_des - self.bl_vel) < self.kierr_vel):
            if (abs(self.bl_vel_des - self.bl_vel) < self.umblal_vel):
                self.kisum_vel = 0.;
            else:
                self.kisum_vel += self.ki_vel * self.error_vel
                self.kisum_vel = self.constrain(self.kisum_vel, -self.kimax_vel, self.kimax_vel)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_vel = 0.

        self.bl_out = self.constrain(self.kp_vel * self.error_vel + self.kisum_vel + self.km_vel * self.bl_vel_des, -self.range_vel, self.range_vel)


    def angCalc(self):

        """MAP FIRST"""
        if self.bl_lec < self.bl_offset:
            self.bl_ang_tmp = self.bl_lec + 1024 - self.bl_offset
        else:
            self.bl_ang_tmp = self.bl_lec - self.bl_offset

        self.bl_ang_tmp = self.map(self.bl_ang_tmp, 0., 1023., 2*pi, 0.)
        self.bl_ang_lst = self.bl_ang_abs
        self.bl_ang_abs = self.bl_ang_tmp
        """MAP FIRST"""

        """LAP CALCULATE"""
        # encuentra si el cambio fue de 0 a 2pi
        if (self.bl_ang_abs > 1.5 * pi and self.bl_ang_lst < 0.5 * pi):
            self.lap -= 1
        # encuetra si el cambio due de 2pi a 0
        if (self.bl_ang_abs < 0.5 * pi and self.bl_ang_lst > 1.5 * pi):
            self.lap += 1
        
        self.bl_ang_lap_lst = self.bl_ang
        self.bl_ang = 2 * pi * self.lap + self.bl_ang_abs
        """LAP CALCULATE"""

        """MAP VEL OUT"""
        self.bl_vel = 10. * (self.bl_ang - self.bl_ang_lap_lst)
        """MAP VEL OUT"""

    def blResetCb(self, data):
        if (data.data == 1):
            self.bl_ang_des = 0

    def blLecCb(self, data):
        self.bl_lec = data.data
        self.angCalc()

        if (abs(self.bl_vel_des) < 0.2):
            # self.bl_ang_des = self.constrain(self.bl_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.bl_ang_des)
        else:
            self.bl_ang_des = self.bl_ang
            self.pid_vel()


    def blDesCb(self, data):
        self.bl_vel_des = data.data
        self.angCalc()

        if (abs(self.bl_vel_des) < 0.2):
            # self.bl_ang_des = self.constrain(self.bl_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.bl_ang_des)
        else:
            self.bl_ang_des = self.bl_ang
            self.pid_vel()


    def update(self):
        self.blOutPub.publish(self.bl_out)
        self.blAngPub.publish(self.bl_ang)
        self.blVelPub.publish(self.bl_vel)


    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':
    """ main """
    bl_node = Bl_node()
    bl_node.spin()
    
