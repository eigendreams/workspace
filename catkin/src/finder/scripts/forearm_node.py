#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

class Forearm_node:

    def __init__(self, node_name_override = 'forearm_node'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("forearm_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        self.forearmOutPub = rospy.Publisher("forearm_out", Int16)
        self.forearmAngPub = rospy.Publisher("forearm_ang", Float32)
        self.forearmVelPub = rospy.Publisher("forearm_vel", Float32)

        #TODO cambiar por parametros
        self.forearm_enc_ana = False
        self.forearm_enc_max = 1023
        self.forearm_offset = 766
        #self.forearm_enc_lim = 100
        #self.forearm_lec_min = 485
        #self.forearm_lec_max = 1004
        #self.forearm_lec_dir = -100

        # PID control parameters
        self.kp_pos = 10
        self.ki_pos = 1
        self.kd_pos = 0
        self.km_pos = 0
        self.umbral_pos = 0.05
        self.range_pos = 50 # Maximo pwm permitido
        self.kierr_pos = 1.2
        self.kimax_pos = 20
        self.kisum_pos = 0
        self.error_pos = 0

        self.kp_vel = 5
        self.ki_vel = 0
        self.kd_vel = 0
        self.km_vel = 0
        self.umbral_vel = 0.5
        self.range_vel = 50 # Maximo pwm permitido
        self.kierr_vel = 2
        self.kimax_vel = 20
        self.kisum_vel = 0
        self.error_vel = 0
        
        # topic variables
        self.forearm_lec = 0
        self.forearm_des = 0
        self.forearm_ang = 0
        self.forearm_ang_des = 0
        self.forearm_vel = 0
        self.forearm_vel_des = 0
        self.forearm_out = 0

        # helper variables
        self.forearm_ang_tmp = 0
        self.forearm_ang_rng = 0
        self.forearm_lec_dst = 0
        self.forearm_ang_lst = 0
        self.forearm_ang_chg = 0
        self.forearm_ang_abs = 0
        self.forearm_ang_lap = 0

        self.forearm_offset_internal = 0.
        self.forearm_ang_tmp_internal = 0      
        self.forearm_ang_lst_internal = 0
        self.forearm_ang_abs_internal = 0

        self.times = 0
        self.init_time = rospy.get_time()

        self.forearmResetSub = rospy.Subscriber("forearm_reset", Int16, self.forearmResetCb)
        self.forearmLecSub = rospy.Subscriber("forearm_lec", Int16, self.forearmLecCb)
        self.forearmDesSub = rospy.Subscriber("forearm_des", Float32, self.forearmDesCb)
        self.offsetSub = rospy.Subscriber("offset", Int16, self.offsetCb)


    def offsetCb(self, data):

        if (data.data == 1):

            self.forearm_offset  = self.forearm_lec
            
            self.forearm_ang_tmp = 0
            self.forearm_ang_lst = 0
            self.forearm_ang_abs = 0

            self.forearm_ang = 0
            self.forearm_ang_lap =  0
            self.forearm_ang_lap_lst = 0


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


    def pid_pos(self):

        if (abs(self.forearm_ang_des - self.forearm_ang) < self.umbral_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.forearm_ang_des - self.forearm_ang
            #print "error " + str(self.error)

        if (abs(self.forearm_ang_des - self.forearm_ang) < self.kierr_pos):
            if (abs(self.forearm_ang_des - self.forearm_ang) < self.umbral_pos):
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_pos = 0.

        self.forearm_out = self.constrain(self.kp_pos * self.error_pos + self.kisum_pos - self.kd_pos * (self.forearm_ang - self.forearm_ang_lst) + self.km_pos * self.forearm_ang_des, -self.range_pos, self.range_pos)


    def pid_vel(self):

        if (abs(self.forearm_vel_des - self.forearm_vel) < self.umbral_vel):
           self.error_vel = 0.
        else:
            self.error_vel = self.forearm_vel_des - self.forearm_vel + 0.
            #print "error " + str(self.error)

        if (abs(self.forearm_vel_des - self.forearm_vel) < self.kierr_vel):
            if (abs(self.forearm_vel_des - self.forearm_vel) < self.umbral_vel):
                self.kisum_vel = 0.;
            else:
                self.kisum_vel += self.ki_vel * self.error_vel
                self.kisum_vel = self.constrain(self.kisum_vel, -self.kimax_vel, self.kimax_vel)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_vel = 0.

        self.forearm_out = self.constrain(self.kp_vel * self.error_vel + self.kisum_vel - self.kd_pos * (self.forearm_ang - self.forearm_ang_lst) + self.km_vel * self.forearm_vel_des, -self.range_vel, self.range_vel)


    def angCalc(self):

        self.times += 1

        self.forearm_offset_internal = self.map(self.forearm_offset, 0., self.forearm_enc_max, 2 * pi, 0)
        self.forearm_ang_tmp_internal = self.map(self.forearm_lec, 0., self.forearm_enc_max, 2 * pi, 0)
        self.forearm_ang_lst_internal = self.forearm_ang_abs_internal
        self.forearm_ang_abs_internal = self.forearm_ang_tmp_internal

        if (self.times > 4):
            # encuentra si el cambio fue de 0 a 2pi
            if (self.forearm_ang_abs_internal > 1.7 * pi and self.forearm_ang_lst_internal < 0.3 * pi):
                self.forearm_ang_lap -= 1
            # encuentra si el cambio fue de 2pi a 0
            if (self.forearm_ang_abs_internal < 0.3 * pi and self.forearm_ang_lst_internal > 1.7 * pi):
                self.forearm_ang_lap += 1

        self.forearm_ang_lap_lst = self.forearm_ang
        self.forearm_ang = 2 * pi * self.forearm_ang_lap + self.forearm_ang_abs_internal - self.forearm_offset_internal 
        self.forearm_vel = 10 * (self.forearm_ang - self.forearm_ang_lap_lst)


    """
    def angCalc(self):
        self.times +=1
        self.forearm_offset_internal = self.map(self.forearm_offset, 0., self.forearm_enc_max, 0., 2 * pi)
        #MAP FIRST
        #
        #if self.forearm_lec < self.forearm_offset:
        #    self.forearm_ang_tmp = self.forearm_lec + 1024 - self.forearm_offset
        #else:
        #    self.forearm_ang_tmp = self.forearm_lec - self.forearm_offset
        #
        #self.forearm_ang_tmp = self.forearm_lec        
        #self.forearm_ang_tmp = self.map(self.forearm_ang_tmp, 0., 1023., 2*pi, 0.)
        #self.forearm_ang_lst = self.forearm_ang_abs
        #self.forearm_ang_abs = self.forearm_ang_tmp

        self.forearm_ang_tmp_internal = self.map(self.forearm_lec, 0., self.forearm_enc_max, 0, 2 * pi)
        self.forearm_ang_lst_internal = self.forearm_ang_abs_internal
        self.forearm_ang_abs_internal = self.forearm_ang_tmp_internal

        #MAP FIRST
        if (self.times > 4):
            # encuentra si el cambio fue de 0 a 2pi
            if (self.forearm_ang_abs_internal > 1.7 * pi and self.forearm_ang_lst_internal < 0.3 * pi):
                self.forearm_ang_lap -= 1
            # encuentra si el cambio fue de 2pi a 0
            if (self.forearm_ang_abs_internal < 0.3 * pi and self.forearm_ang_lst_internal > 1.7 * pi):
                self.forearm_ang_lap += 1        

        #LAP CALCULATE
        
        self.forearm_ang_lap_lst = self.forearm_ang
        self.forearm_ang = 2 * pi * self.lap + self.forearm_ang_abs
        #LAP CALCULATE

        #MAP VEL OUT
        self.forearm_vel =  (self.forearm_ang - self.forearm_ang_lap_lst)
        #MAP VEL OUT
    """


    def forearmResetCb(self, data):
        
        if (data.data == 1):
            self.forearm_ang_des = 0


    def forearmLecCb(self, data):

        self.forearm_lec = data.data
        self.angCalc()

        # if (abs(self.forearm_vel_des) < 0.1):
        #     # self.forearm_ang_des = self.constrain(self.forearm_ang_des, 0, 1000)
        #     self.pid_pos()
        #     # print "angdes " + str(self.forearm_ang_des)
        # else:
        #     self.forearm_ang_des = self.forearm_ang
        #     self.pid_vel()


    def forearmDesCb(self, data):

        self.forearm_vel_des = self.constrain(data.data, -80, 80)
        self.angCalc()

        # if (abs(self.forearm_vel_des) < 0.1):
        #     # self.forearm_ang_des = self.constrain(self.forearm_ang_des, 0, 1000)
        #     self.pid_pos()
        #     # print "angdes " + str(self.forearm_ang_des)
        # else:
        #     self.forearm_ang_des = self.forearm_ang
        #     self.pid_vel()


    def update(self):

        self.forearmOutPub.publish(self.forearm_vel_des)
        #self.forearmOutPub.publish(self.forearm_out)
        self.forearmAngPub.publish(self.forearm_ang)
        self.forearmVelPub.publish(self.forearm_vel)


    def spin(self):

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':

    """ main """
    forearm_node = Forearm_node()
    forearm_node.spin() 