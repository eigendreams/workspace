#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

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
        self.umbral_pos = 0.5
        self.range_pos = 60 # Maximo pwm permitido
        self.kierr_pos = 5
        self.kimax_pos = 30
        self.kisum_pos = 0
        self.error_pos = 0

        self.kp_vel = 5
        self.ki_vel = 2
        self.kd_vel = 0
        self.km_vel = 0
        self.umbral_vel = 0.5
        self.range_vel = 60 # Maximo pwm permitido
        self.kierr_vel = 1
        self.kimax_vel = 30
        self.kisum_vel = 0
        self.error_vel = 0

        # topic variables
        self.bl_lec = 0
        self.bl_des = 0
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
        self.bl_ang_lap = 0

        self.bl_offset_internal = 0.
        self.bl_ang_tmp_internal = 0      
        self.bl_ang_lst_internal = 0
        self.bl_ang_abs_internal = 0

        self.times = 0
        self.times_lec = 0
        self.init_time = rospy.get_time()

        self.bl_save = 0

        self.blResetSub = rospy.Subscriber("bl_reset", Int16, self.blResetCb)
        self.blLecSub = rospy.Subscriber("bl_lec", Int16, self.blLecCb)
        self.blDesSub = rospy.Subscriber("bl_des", Float32, self.blDesCb)
        self.offsetSub = rospy.Subscriber("offset", Int16, self.offsetCb)


    def offsetCb(self, data):

        if (data.data == 1):

            self.bl_offset  = self.bl_lec
            
            self.bl_ang_tmp = 0
            self.bl_ang_lst = 0
            self.bl_ang_abs = 0

            self.bl_ang = 0
            self.bl_ang_lap =  0
            self.bl_ang_lap_lst = 0


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

        if (abs(self.bl_ang_des - self.bl_ang) < self.umbral_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.bl_ang_des - self.bl_ang
            #print "error " + str(self.error)

        if (abs(self.bl_ang_des - self.bl_ang) < self.kierr_pos):
            if (abs(self.bl_ang_des - self.bl_ang) < self.umbral_pos):
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_pos = 0.

        self.bl_out = self.constrain(self.kp_pos * self.error_pos + self.kisum_pos - self.kd_pos * (self.bl_ang - self.bl_ang_lst) + self.km_pos * self.bl_ang_des, -self.range_pos, self.range_pos)


    def pid_vel(self):

        if (abs(self.bl_vel_des - self.bl_vel) < self.umbral_vel):
           self.error_vel = 0.
        else:
            self.error_vel = self.bl_vel_des - self.bl_vel + 0.
            #print "error " + str(self.error)

        if (abs(self.bl_vel_des - self.bl_vel) < self.kierr_vel):
            if (abs(self.bl_vel_des - self.bl_vel) < self.umbral_vel):
                self.kisum_vel = 0.;
            else:
                self.kisum_vel += self.ki_vel * self.error_vel
                self.kisum_vel = self.constrain(self.kisum_vel, -self.kimax_vel, self.kimax_vel)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_vel = 0.

        self.bl_out = self.constrain(self.kp_vel * self.error_vel + self.kisum_vel - self.kd_pos * (self.bl_ang - self.bl_ang_lst) + self.km_vel * self.bl_vel_des, -self.range_vel, self.range_vel)


    def angCalc(self):

        self.times += 1

        self.bl_offset_internal = self.map(self.bl_offset, 0., self.bl_enc_max, 2 * pi, 0)
        self.bl_ang_tmp_internal = self.map(self.bl_lec, 0., self.bl_enc_max, 2 * pi, 0)
        self.bl_ang_lst_internal = self.bl_ang_abs_internal
        self.bl_ang_abs_internal = self.bl_ang_tmp_internal

        if (self.times > 4):
            # encuentra si el cambio fue de 0 a 2pi
            if (self.bl_ang_abs_internal > 1.7 * pi and self.bl_ang_lst_internal < 0.3 * pi):
                self.bl_ang_lap -= 1
            # encuentra si el cambio fue de 2pi a 0
            if (self.bl_ang_abs_internal < 0.3 * pi and self.bl_ang_lst_internal > 1.7 * pi):
                self.bl_ang_lap += 1

        self.bl_ang_lap_lst = self.bl_ang
        self.bl_ang = 2 * pi * self.bl_ang_lap + self.bl_ang_abs_internal - self.bl_offset_internal 
        self.bl_vel = 10 * (self.bl_ang - self.bl_ang_lap_lst)


    """
    def angCalc(self):

        # #MAP FIRST
        # 
        # if self.bl_lec < self.bl_offset:
        #     self.bl_ang_tmp = self.bl_lec + 1024 - self.bl_offset
        # else:
        #     self.bl_ang_tmp = self.bl_lec - self.bl_offset
        # 

        # 
        # self.bl_ang_tmp = self.bl_lec        
        # self.bl_ang_tmp = self.map(self.bl_ang_tmp, 0., 1023., 2*pi, 0.)
        # self.bl_ang_lst = self.bl_ang_abs
        # self.bl_ang_abs = self.bl_ang_tmp
        # 
        # MAP FIRST

        # LAP CALCULATE
        # 
        # # encuentra si el cambio fue de 0 a 2pi
        # if (self.bl_ang_abs > 1.8 * pi and self.bl_ang_lst < 0.2 * pi):
        #     self.lap -= 1
        # # encuetra si el cambio due de 2pi a 0
        # if (self.bl_ang_abs < 0.2 * pi and self.bl_ang_lst > 1.8 * pi):
        #     self.lap += 1
        # 

        self.bl_ang_tmp_internal = self.map(self.bl_lec, 0., self.bl_enc_max, 0, 2 * pi)
        self.bl_ang_lst_internal = self.bl_ang_abs_internal
        self.bl_ang_abs_internal = self.bl_ang_tmp_internal
        if (self.times > 4):
            # encuentra si el cambio fue de 0 a 2pi
            if (self.bl_ang_abs_internal > 1.7 * pi and self.bl_ang_lst_internal < 0.3 * pi):
                self.bl_ang_lap -= 1
            # encuentra si el cambio fue de 2pi a 0
            if (self.bl_ang_abs_internal < 0.3 * pi and self.bl_ang_lst_internal > 1.7 * pi):
                self.bl_ang_lap += 1        

        self.bl_ang_lap_lst = self.bl_ang
        self.bl_ang = 2 * pi * self.lap + self.bl_ang_abs
        

        #LAP CALCULATE

        #MAP VEL OUT
        self.bl_vel = 10. * (self.bl_ang - self.bl_ang_lap_lst)
        #MAP VEL OUT
    """


    def blResetCb(self, data):
        
        if (data.data == 1):
            self.bl_ang_des = 0


    """
    def blLecCb(self, data):
        
        self.bl_lec = data.data
        self.angCalc()
        self.pid()


    def blDesCb(self, data):

        self.bl_des = data.data
        self.angCalc()
        self.pid()
    """


    def blLecCb(self, data):

        self.times_lec += 1
        if (self.times_lec < 4):
            self.offset = data.data

        self.bl_lec = data.data
        self.angCalc()

        if (abs(self.bl_vel_des) < 0.1):
            # self.bl_ang_des = self.constrain(self.bl_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.bl_ang_des)
        else:
            self.bl_ang_des = self.bl_ang
            self.pid_vel()


    def blDesCb(self, data):

        self.bl_vel_des = data.data
        self.angCalc()

        if (abs(self.bl_vel_des) < 0.1):
            # self.bl_ang_des = self.constrain(self.bl_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.bl_ang_des)
        else:
            self.bl_ang_des = self.bl_ang
            self.pid_vel()


    def update(self):

        self.bl_save = abs(self.bl_out) + self.bl_save * 0.95 #MAXIMO DE 70 POR 35 (1 + 35 * 0.5 + ...)
        self.range_pos = 60 - self.bl_save / 40
        self.range_vel = 60 - self.bl_save / 40

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
