#!/usr/bin/env python
# -*- coding: utf8 -*-

"""
Esta clase representa la plantilla de un nodo de control de un motor por medio de un bucle PID

Nodo:   motor_pid_loop
    Suscriptores:   motordes        El valor de velocidad deseado del motor
                    motorlec        El valor de lectura del encoder digital absoluto en el eje del motor
    Publicadores:   motorout        El valor de salida deseada PWM del motor

Esta clase NO puede tener información sobre la "verdadera" posicion del motor, porque los encoders no 
permiten obtener tal información, al girar mas de una vuelta en el movimiento del pendulo, otro nodo
externo debera hacer esto por medio de la IMU, el proposito de esta clase es solamente el control de velocidad
del motor y el mantenimiento de posicion por debajo de un umbral minimo de velocidad
"""

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

class Motor_pid_loop:

    def __init__(self, node_name_override = 'motor_pid_loop'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("motor_pid_loop starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)
        self.init_time = self.rospy.get_time()

        #TODO cambiar por parametros
        self.motor_enc_ana = False
        self.motor_enc_max = 1023
        self.motor_offset  = 0

        # PID control parameters
        self.kp_pos = 70
        self.ki_pos = 5.
        self.kd_pos = 0.
        self.km_pos = 0.
        self.umbral_pos = 0.1
        self.range_pos = 80.    # Maximo pwm permitido por defecto
        self.kierr_pos = 2
        self.kimax_pos = 25.
        self.kisum_pos = 0.
        self.error_pos = 0.

        self.kp_vel = 70
        self.ki_vel = 5.
        self.kd_vel = 0.
        self.km_vel = 0.
        self.umbral_vel = 0.1
        self.range_vel = 80.    # Maximo pwm permitido por defecto
        self.kierr_vel = 2
        self.kimax_vel = 25.
        self.kisum_vel = 0.
        self.error_vel = 0.

        # topic variables
        self.motor_lec = 0
        self.motor_des = 0
        self.motor_ang = 0
        self.motor_ang_des = 0
        self.motor_vel = 0
        self.motor_vel_des = 0
        self.motor_out = 0

        # helper variables
        self.motor_ang_tmp = 0
        self.motor_ang_rng = 0
        self.motor_lec_dst = 0
        self.motor_ang_lst = 0
        self.motor_ang_chg = 0
        self.motor_ang_abs = 0
        self.motor_ang_lap = 0

        self.motor_offset_internal = 0.
        self.motor_ang_tmp_internal = 0      
        self.motor_ang_lst_internal = 0
        self.motor_ang_abs_internal = 0

        self.times = 0
        self.times_lec = 0

        self.motorout = rospy.Publisher("motorout", Int16)                      # PWM VAL 
        self.motorang = rospy.Publisher("motorang", Float32)                    # IN RAD
        self.motorvel = rospy.Publisher("motorvel", Float32)                    # IN RAD/S
        self.motordes = rospy.Subscriber("motordes", Int16, self.motordescb)    # desired speed OF ENCODERS IN RAD/S
        self.motorlec = rospy.Subscriber("motorlec", Int16, self.motorleccb)    # BINARY ENCODER LECTURE

    def angCalc(self):

        self.times += 1

        self.motor_offset_internal  = self.map(self.motor_offset,   0., 1023., 2 * pi, 0.)
        self.motor_ang_tmp_internal = self.map(self.motor_lec,      0., 1023., 2 * pi, 0.)
        self.motor_ang_lst_internal = self.motor_ang_abs_internal
        self.motor_ang_abs_internal = self.motor_ang_tmp_internal

        # begin processing after first second
        if (self.times > 10):
            # encuentra si el cambio fue de 0 a 2pi
            if (self.motor_ang_abs_internal > 1.8 * pi and self.motor_ang_lst_internal < 0.2 * pi):
                self.motor_ang_lap -= 1
            # encuentra si el cambio fue de 2pi a 0
            if (self.motor_ang_abs_internal < 0.2 * pi and self.motor_ang_lst_internal > 1.8 * pi):
                self.motor_ang_lap += 1

        self.motor_ang_lap_lst  = self.motor_ang
        self.motor_ang          = 2 * pi * self.motor_ang_lap + self.motor_ang_abs_internal - self.motor_offset_internal 
        self.motor_vel          = 10 * (self.motor_ang - self.motor_ang_lap_lst)

    def motorleccb(self, data):

        # autoset the offset within the first second of operation
        self.times_lec += 1
        if (self.times_lec < 10):
            self.motor_offset = data.data
        self.motor_lec = data.data

        # calculate the "angle"
        self.angCalc()

        # decide the control action
        if (abs(self.motor_vel_des) < 0.1):
            self.pid_pos()
        else:
            self.motor_ang_des = self.motor_ang
            self.pid_vel()

    def motordescb(self, data):

        self.motor_vel_des = data.data

        # calculate the "angle"
        self.angCalc()

        if (abs(self.motor_vel_des) < 0.1):
            self.pid_pos()
        else:
            self.motor_ang_des = self.motor_ang
            self.pid_vel()

    ###########################################################################
    def map(self, x, in_min, in_max, out_min, out_max):

        return (x - in_min) * (out_max - out_min + 0.) / (in_max - in_min + 0.) + out_min

    def millis(self):

        return int(1000 * (self.rospy.get_time() - self.init_time))

    def constrain(self, x, min, max):

        if (x > max):
            return max
        if (x < min):
            return min
        return x

    def sign(self, a):

        return (a > 0) - (a < 0)
    ###########################################################################

    def pid_pos(self):

        if (abs(self.motor_ang_des - self.motor_ang) < self.umbral_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.motor_ang_des - self.motor_ang
            #print "error " + str(self.error)

        if (abs(self.motor_ang_des - self.motor_ang) < self.kierr_pos):
            if (abs(self.motor_ang_des - self.motor_ang) < self.umbral_pos):
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_pos = 0.

        self.motor_out = self.constrain(self.kp_pos * self.error_pos + self.kisum_pos - self.kd_pos * (self.motor_ang - self.motor_ang_lst) + self.km_pos * self.motor_ang_des, -self.range_pos, self.range_pos)

    def pid_vel(self):

        if (abs(self.motor_vel_des - self.motor_vel) < self.umbral_vel):
           self.error_vel = 0.
        else:
            self.error_vel = self.motor_vel_des - self.motor_vel + 0.
            #print "error " + str(self.error)

        if (abs(self.motor_vel_des - self.motor_vel) < self.kierr_vel):
            if (abs(self.motor_vel_des - self.motor_vel) < self.umbral_vel):
                self.kisum_vel = 0.;
            else:
                self.kisum_vel += self.ki_vel * self.error_vel
                self.kisum_vel = self.constrain(self.kisum_vel, -self.kimax_vel, self.kimax_vel)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_vel = 0.

        self.motor_out = self.constrain(self.kp_vel * self.error_vel + self.kisum_vel - self.kd_pos * (self.motor_ang - self.motor_ang_lst) + self.km_vel * self.motor_vel_des, -self.range_vel, self.range_vel)

    def update(self):

        self.motorout.publish(self.motor_out)
        self.motorang.publish(self.motor_ang)
        self.motorvel.publish(self.motor_vel)

    def spin(self):

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

if __name__ == '__main__':

    """ main """
    motor_pid_loop = Motor_pid_loop()
    motor_pid_loop.spin() 
