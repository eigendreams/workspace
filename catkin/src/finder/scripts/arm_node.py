#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

class Arm_node:

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
        self.kp_pos = 70
        self.ki_pos = 0
        self.kd_pos = 0
        self.km_pos = 0
        self.umbral_pos = 0.5
        self.range_pos = 70 # Maximo pwm permitido
        self.kierr_pos = 1.2
        self.kimax_pos = 10
        self.kisum_pos = 0
        self.error_pos = 0

        self.kp_vel = 70
        self.ki_vel = 0
        self.kd_vel = 0
        self.km_vel = 0
        self.umbral_vel = 0.5
        self.range_vel = 70 # Maximo pwm permitido
        self.kierr_vel = 2
        self.kimax_vel = 10
        self.kisum_vel = 0
        self.error_vel = 0

        # topic variables
        self.arm_lec = 0
        self.arm_des = 0
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
        self.arm_ang_lap = 0

        self.arm_offset_internal = 0.
        self.arm_ang_tmp_internal = 0      
        self.arm_ang_lst_internal = 0
        self.arm_ang_abs_internal = 0

        self.times = 0
        self.times_lec = 0
        self.init_time = rospy.get_time()

        self.arm_save = 0

        self.armResetSub = rospy.Subscriber("arm_reset", Int16, self.armResetCb)
        self.armLecSub = rospy.Subscriber("arm_lec", Int16, self.armLecCb)
        self.armDesSub = rospy.Subscriber("arm_des", Float32, self.armDesCb)
        self.offsetSub = rospy.Subscriber("offset", Int16, self.offsetCb)


    def offsetCb(self, data):

        if (data.data == 1):

            self.arm_offset  = self.arm_lec
            
            self.arm_ang_tmp = 0
            self.arm_ang_lst = 0
            self.arm_ang_abs = 0

            self.arm_ang = 0
            self.arm_ang_lap =  0
            self.arm_ang_lap_lst = 0


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

        if (abs(self.arm_ang_des - self.arm_ang) < self.umbral_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.arm_ang_des - self.arm_ang
            #print "error " + str(self.error)

        if (abs(self.arm_ang_des - self.arm_ang) < self.kierr_pos):
            if (abs(self.arm_ang_des - self.arm_ang) < self.umbral_pos):
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_pos = 0.

        self.arm_out = self.constrain(self.kp_pos * self.error_pos + self.kisum_pos - self.kd_pos * (self.arm_ang - self.arm_ang_lst) + self.km_pos * self.arm_ang_des, -self.range_pos, self.range_pos)


    def pid_vel(self):

        if (abs(self.arm_vel_des - self.arm_vel) < self.umbral_vel):
           self.error_vel = 0.
        else:
            self.error_vel = self.arm_vel_des - self.arm_vel + 0.
            #print "error " + str(self.error)

        if (abs(self.arm_vel_des - self.arm_vel) < self.kierr_vel):
            if (abs(self.arm_vel_des - self.arm_vel) < self.umbral_vel):
                self.kisum_vel = 0.;
            else:
                self.kisum_vel += self.ki_vel * self.error_vel
                self.kisum_vel = self.constrain(self.kisum_vel, -self.kimax_vel, self.kimax_vel)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_vel = 0.

        self.arm_out = self.constrain(self.kp_vel * self.error_vel + self.kisum_vel - self.kd_pos * (self.arm_ang - self.arm_ang_lst) + self.km_vel * self.arm_vel_des, -self.range_vel, self.range_vel)


    def angCalc(self):

        self.times += 1

        self.arm_offset_internal = self.map(self.arm_offset, 0., self.arm_enc_max, 2 * pi, 0)
        self.arm_ang_tmp_internal = self.map(self.arm_lec, 0., self.arm_enc_max, 2 * pi, 0)
        self.arm_ang_lst_internal = self.arm_ang_abs_internal
        self.arm_ang_abs_internal = self.arm_ang_tmp_internal

        if (self.times > 4):
            # encuentra si el cambio fue de 0 a 2pi
            if (self.arm_ang_abs_internal > 1.7 * pi and self.arm_ang_lst_internal < 0.3 * pi):
                self.arm_ang_lap -= 1
            # encuentra si el cambio fue de 2pi a 0
            if (self.arm_ang_abs_internal < 0.3 * pi and self.arm_ang_lst_internal > 1.7 * pi):
                self.arm_ang_lap += 1

        self.arm_ang_lap_lst = self.arm_ang
        self.arm_ang = 2 * pi * self.arm_ang_lap + self.arm_ang_abs_internal - self.arm_offset_internal 
        self.arm_vel = 10 * (self.arm_ang - self.arm_ang_lap_lst)


    """
    def angCalc(self):
        self.times +=1
        self.arm_offset_internal = self.map(self.arm_offset, 0., self.arm_enc_max, 0., 2 * pi)

        #MAP FIRST
        #
        #if self.arm_lec < self.arm_offset:
        #    self.arm_ang_tmp = self.arm_lec + 1024 - self.arm_offset
        #else:
        #    self.arm_ang_tmp = self.arm_lec - self.arm_offset
        #

        self.arm_ang_tmp_internal = self.map(self.arm_lec, 0., self.arm_enc_max, 0, 2 * pi)
        self.arm_ang_lst_internal = self.arm_ang_abs_internal
        self.arm_ang_abs_internal = self.arm_ang_tmp_internal
        if (self.times > 4):
            # encuentra si el cambio fue de 0 a 2pi
            if (self.arm_ang_abs_internal > 1.7 * pi and self.arm_ang_lst_internal < 0.3 * pi):
                self.arm_ang_lap -= 1
            # encuentra si el cambio fue de 2pi a 0
            if (self.arm_ang_abs_internal < 0.3 * pi and self.arm_ang_lst_internal > 1.7 * pi):
                self.arm_ang_lap += 1        
        #
        #self.arm_ang_tmp = self.arm_lec        
        #self.arm_ang_tmp = self.map(self.arm_ang_tmp, 0., 1023., 2*pi, 0.)
        #self.arm_ang_lst = self.arm_ang_abs
        #self.arm_ang_abs = self.arm_ang_tmp
        #
        
        self.arm_ang_lap_lst = self.arm_ang
        self.arm_ang = 2 * pi * self.arm_ang_lap + self.arm_ang_lap_lst
        #LAP CALCULATE

        #MAP VEL OUT
        self.arm_vel = (self.arm_ang - self.arm_ang_lap_lst)
        #MAP VEL OUT
    """


    def armResetCb(self, data):
        
        if (data.data == 1):
            self.arm_ang_des = 0


    def armLecCb(self, data):

        self.times_lec += 1
        if (self.times_lec < 4):
            self.offset = data.data

        self.arm_lec = data.data
        self.angCalc()

        if (abs(self.arm_vel_des) < 0.1):
            # self.arm_ang_des = self.constrain(self.arm_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.arm_ang_des)
        else:
            self.arm_ang_des = self.arm_ang
            self.pid_vel()


    def armDesCb(self, data):

        self.arm_vel_des = self.constrain(data.data, -100, 100) / 100.
        self.angCalc()

        if (abs(self.arm_vel_des) < 0.1):
            # self.arm_ang_des = self.constrain(self.arm_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.arm_ang_des)
        else:
            self.arm_ang_des = self.arm_ang
            self.pid_vel()


    def update(self):

        self.arm_save = abs(self.arm_out) + self.arm_save * 0.95 #MAXIMO DE 70 POR 35 (1 + 35 * 0.5 + ...)
        self.range_pos = 70 - self.arm_save / 40
        self.range_vel = 70 - self.arm_save / 40

        #print(str(self.arm_save))

        self.armOutPub.publish(self.arm_vel_des * 100)
        #self.armOutPub.publish(self.arm_out)
        self.armAngPub.publish(self.arm_ang)
        self.armVelPub.publish(self.arm_vel)


    def spin(self):

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':

    """ main """
    arm_node = Arm_node()
    arm_node.spin()
