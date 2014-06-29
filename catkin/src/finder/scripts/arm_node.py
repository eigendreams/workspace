#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

"""
El proposito de esta clase es servir de nodo de control de bajo nivel para el motor de la arm del robot, el microcontrolador
publica un topico de tipo Int16 llamada "arm_lec" y consume otro topico de tipo Int16 llamado "arm_out". arm_lec es la
lectura sin procesar del encoder, y arm_out es el valor de salida PWM del motor, que puede ser un valor de -100 a 100. El
procesamiento de los datos del encoder no es tan trivial, pero se incluyen las funciones necesarias para realizarlo.

Esta clase debe suscrivirse a un topico Int16 llamado "arm_des" y publicar dos topicos Int16 llamados "arm_ang" y 
"arm_vel" que contengan informacion del angulo y la velocidad. Internamente, el control se realiza soarme el valor del
angulo, haciendo un match PID entre arm_des y arm_ang.

ESTE MOTOR SE CONTROLA A BAJO NIVEL POR POSICION
"""

class Fl_node:

    def __init__(self, node_name_override = 'arm_node'):
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("arm_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        self.armOutPub = rospy.Publisher("arm_out", Int16)
        self.armAngPub = rospy.Publisher("arm_ang", Float32)
        self.armVelPub = rospy.Publisher("arm_vel", Float32)

        #TODO cambiar por parametros
        self.arm_enc_ana = False
        self.arm_enc_max = 1023
        self.arm_offset = 766
        #self.arm_enc_lim = 100
        #self.arm_lec_min = 485
        #self.arm_lec_max = 1004
        #self.arm_lec_dir = -100

        # PID control parameters
        self.kp_pos = 10
        self.ki_pos = 10
        self.kd_pos = 0
        self.km_pos = 0
        self.umarmal_pos = 0.1
        self.range_pos = 50 # Maximo pwm permitido
        self.kierr_pos = 1.2
        self.kimax_pos = 50
        self.kisum_pos = 0
        self.error_pos = 0

        self.kp_vel = 5
        self.ki_vel = 2
        self.kd_vel = 0
        self.km_vel = 0
        self.umarmal_vel = 0.5
        self.range_vel = 50 # Maximo pwm permitido
        self.kierr_vel = 1
        self.kimax_vel = 50
        self.kisum_vel = 0
        self.error_vel = 0

        # topic variables
        self.arm_lec = 0
        #self.arm_des = 0
        self.arm_ang = 0
        self.arm_ang_des = 0   
        self.arm_vel = 0
        self.arm_vel_des = 0
        self.arm_out = 0

        # helper variables
        self.arm_ang_tmp = 0
        self.arm_ang_rng = 0
        self.arm_lec_dst = 0
        self.arm_ang_lst = 0
        self.arm_ang_chg = 0

        self.arm_ang_abs = 0
        self.lap = 0

        # inits
        self.init_time = rospy.get_time()
        #self.angInit()

        self.init_armag = False;

        self.armResetSub = rospy.Subscriber("arm_reset", Int16, self.armResetCb)
        self.armLecSub = rospy.Subscriber("arm_lec", Int16, self.armLecCb)
        self.armDesSub = rospy.Subscriber("arm_des", Float32, self.armDesCb)
        self.offsetSub = rospy.Subscriber("offset", Int16, self.offsetCb)

    def offsetCb(self, data):

        if(data.data == 1):

            self.arm_offset  = self.arm_lec
            
            self.arm_ang_tmp = 0
            self.arm_ang_lst = 0
            self.arm_ang_abs = 0

            self.arm_ang = 0
            self.arm_ang_lap =  0
            self.arm_ang_lap_lst = 0


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

        if (abs(self.arm_ang_des - self.arm_ang) <= self.umarmal_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.arm_ang_des - self.arm_ang
            #print "error " + str(self.error)


        if (abs(self.arm_ang_des - self.arm_ang) < self.kierr_pos):
            if (abs(self.arm_ang_des - self.arm_ang) < self.umarmal_pos):
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_pos = 0.

        self.arm_out = self.constrain(self.kp_pos * self.error_pos + self.kisum_pos + self.km_pos * self.arm_ang_des, -self.range_pos, self.range_pos)

    def pid_vel(self):

        if (abs(self.arm_vel_des - self.arm_vel) <= self.umarmal_vel):
           self.error_vel = 0.
        else:
            self.error_vel = self.arm_vel_des - self.arm_vel + 0.
            #print "error " + str(self.error)


        if (abs(self.arm_vel_des - self.arm_vel) < self.kierr_vel):
            if (abs(self.arm_vel_des - self.arm_vel) < self.umarmal_vel):
                self.kisum_vel = 0.;
            else:
                self.kisum_vel += self.ki_vel * self.error_vel
                self.kisum_vel = self.constrain(self.kisum_vel, -self.kimax_vel, self.kimax_vel)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_vel = 0.

        self.arm_out = self.constrain(self.kp_vel * self.error_vel + self.kisum_vel + self.km_vel * self.arm_vel_des, -self.range_vel, self.range_vel)


    def angCalc(self):

        """MAP FIRST"""
        """
        if self.arm_lec < self.arm_offset:
            self.arm_ang_tmp = self.arm_lec + 1024 - self.arm_offset
        else:
            self.arm_ang_tmp = self.arm_lec - self.arm_offset
        """
        
        self.arm_ang_tmp = self.map(self.arm_ang_tmp, 0., 1023., 2*pi, 0.)
        self.arm_ang_lst = self.arm_ang_abs
        self.arm_ang_abs = self.arm_ang_tmp
        """MAP FIRST"""

        """LAP CALCULATE"""
        # encuentra si el cambio fue de 0 a 2pi
        if (self.arm_ang_abs > 1.8 * pi and self.arm_ang_lst < 0.2 * pi):
            self.lap -= 1
        # encuetra si el cambio due de 2pi a 0
        if (self.arm_ang_abs < 0.2 * pi and self.arm_ang_lst > 1.8 * pi):
            self.lap += 1
        
        self.arm_ang_lap_lst = self.arm_ang
        self.arm_ang = 2 * pi * self.lap + self.arm_ang_abs
        """LAP CALCULATE"""

        """MAP VEL OUT"""
        self.arm_vel = 10. * (self.arm_ang - self.arm_ang_lap_lst)
        """MAP VEL OUT"""

    def armResetCb(self, data):
        if (data.data == 1):
            self.arm_ang_des = 0

    def armLecCb(self, data):
        self.arm_lec = data.data
        self.angCalc()

        if (abs(self.arm_vel_des) < 0.2):
            # self.arm_ang_des = self.constrain(self.arm_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.arm_ang_des)
        else:
            self.arm_ang_des = self.arm_ang
            self.pid_vel()


    def armDesCb(self, data):
        self.arm_vel_des = data.data
        self.angCalc()

        if (abs(self.arm_vel_des) < 0.2):
            # self.arm_ang_des = self.constrain(self.arm_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.arm_ang_des)
        else:
            self.arm_ang_des = self.arm_ang
            self.pid_vel()


    def update(self):
        self.armOutPub.publish(self.arm_out)
        self.armAngPub.publish(self.arm_ang)
        self.armVelPub.publish(self.arm_vel)


    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':
    """ main """
    arm_node = Fl_node()
    arm_node.spin()
    
