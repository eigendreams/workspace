#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

class Fr_node:

    def __init__(self, node_name_override = 'fr_node'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("fr_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        self.frOutPub = rospy.Publisher("fr_out", Int16)
        self.frAngPub = rospy.Publisher("fr_ang", Float32)
        self.frVelPub = rospy.Publisher("fr_vel", Float32)

        #TODO cambiar por parametros
        self.fr_enc_ana = False
        self.fr_enc_max = 1023
        self.fr_offset = 123
        #self.fr_enc_lim = 100
        #self.fr_lec_min = 485
        #self.fr_lec_max = 1004
        #self.fr_lec_dir = -100

        # PID control parameters
        self.kp_pos = 10
        self.ki_pos = 1
        self.kd_pos = 0
        self.km_pos = 0
        self.umbral_pos = 0.5
        self.range_pos = 50 # Maximo pwm permitido
        self.kierr_pos = 2
        self.kimax_pos = 25
        self.kisum_pos = 0
        self.error_pos = 0

        self.kp_vel = 5
        self.ki_vel = 1
        self.kd_vel = 0
        self.km_vel = 0
        self.umbral_vel = 0.5
        self.range_vel = 50 # Maximo pwm permitido
        self.kierr_vel = 1
        self.kimax_vel = 25
        self.kisum_vel = 0
        self.error_vel = 0

        # topic variables
        self.fr_lec = 0
        self.fr_des = 0
        self.fr_ang = 0
        self.fr_ang_des = 0
        self.fr_vel = 0
        self.fr_vel_des = 0
        self.fr_out = 0

        # helper variables
        self.fr_ang_tmp = 0
        self.fr_ang_rng = 0
        self.fr_lec_dst = 0
        self.fr_ang_lst = 0
        self.fr_ang_chg = 0
        self.fr_ang_abs = 0
        self.fr_ang_lap = 0

        self.fr_offset_internal = 0.
        self.fr_ang_tmp_internal = 0      
        self.fr_ang_lst_internal = 0
        self.fr_ang_abs_internal = 0

        self.times = 0
        self.times_lec = 0
        self.init_time = rospy.get_time()

        self.fr_save = 0

        self.frResetSub = rospy.Subscriber("fr_reset", Int16, self.frResetCb)
        self.frLecSub = rospy.Subscriber("fr_lec", Int16, self.frLecCb)
        self.frDesSub = rospy.Subscriber("fr_des", Float32, self.frDesCb)
        self.offsetSub = rospy.Subscriber("offset", Int16, self.offsetCb)


    def offsetCb(self, data):

        if (data.data == 1):

            self.fr_offset  = self.fr_lec
            
            self.fr_ang_tmp = 0
            self.fr_ang_lst = 0
            self.fr_ang_abs = 0

            self.fr_ang = 0
            self.fr_ang_lap =  0
            self.fr_ang_lap_lst = 0


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

    """
    def pid(self):

        if (abs(self.fr_des - self.fr_ang) < self.umbral_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.fr_des - self.fr_ang

        if (abs(self.fr_des - self.fr_ang) < self.kierr_pos):
            if (abs(self.fr_des - self.fr_ang) < self.umbral_pos):
                #pass
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
        else:
            self.kisum_pos = 0.

        self.fr_out = -self.constrain(self.kp_pos * self.error_pos + self.kisum_pos - self.kd_pos * (self.fr_ang - self.fr_ang_lst) + self.km_pos * self.fr_des, -self.range_pos, self.range_pos)
    """


    def pid_pos(self):

        if (abs(self.fr_ang_des - self.fr_ang) < self.umbral_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.fr_ang_des - self.fr_ang
            #print "error " + str(self.error)

        if (abs(self.fr_ang_des - self.fr_ang) < self.kierr_pos):
            if (abs(self.fr_ang_des - self.fr_ang) < self.umbral_pos):
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_pos = 0.

        self.fr_out = self.constrain(self.kp_pos * self.error_pos + self.kisum_pos - self.kd_pos * (self.fr_ang - self.fr_ang_lst) + self.km_pos * self.fr_ang_des, -self.range_pos, self.range_pos)


    def pid_vel(self):

        if (abs(self.fr_vel_des - self.fr_vel) < self.umbral_vel):
           self.error_vel = 0.
        else:
            self.error_vel = self.fr_vel_des - self.fr_vel + 0.
            #print "error " + str(self.error)

        if (abs(self.fr_vel_des - self.fr_vel) < self.kierr_vel):
            if (abs(self.fr_vel_des - self.fr_vel) < self.umbral_vel):
                self.kisum_vel = 0.;
            else:
                self.kisum_vel += self.ki_vel * self.error_vel
                self.kisum_vel = self.constrain(self.kisum_vel, -self.kimax_vel, self.kimax_vel)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_vel = 0.

        self.fr_out = self.constrain(self.kp_vel * self.error_vel + self.kisum_vel - self.kd_pos * (self.fr_ang - self.fr_ang_lst) + self.km_vel * self.fr_vel_des, -self.range_vel, self.range_vel)


    def angCalc(self):

        self.times += 1

        self.fr_offset_internal = self.map(self.fr_offset, 0., self.fr_enc_max, 2 * pi, 0)
        self.fr_ang_tmp_internal = self.map(self.fr_lec, 0., self.fr_enc_max, 2 * pi, 0)
        self.fr_ang_lst_internal = self.fr_ang_abs_internal
        self.fr_ang_abs_internal = self.fr_ang_tmp_internal

        if (self.times > 4):
            # encuentra si el cambio fue de 0 a 2pi
            if (self.fr_ang_abs_internal > 1.7 * pi and self.fr_ang_lst_internal < 0.3 * pi):
                self.fr_ang_lap -= 1
            # encuentra si el cambio fue de 2pi a 0
            if (self.fr_ang_abs_internal < 0.3 * pi and self.fr_ang_lst_internal > 1.7 * pi):
                self.fr_ang_lap += 1

        self.fr_ang_lap_lst = self.fr_ang
        self.fr_ang = 2 * pi * self.fr_ang_lap + self.fr_ang_abs_internal - self.fr_offset_internal 
        self.fr_vel = 10 * (self.fr_ang - self.fr_ang_lap_lst)


    """
    def angCalc(self):

        self.fr_ang_tmp = self.fr_lec        
        self.fr_ang_tmp = self.map(self.fr_ang_tmp, 0., 1023., 2*pi, 0.)
        self.fr_ang_lst = self.fr_ang_abs
        self.fr_ang_abs = self.fr_ang_tmp

        # encuentra si el cambio fue de 0 a 2pi
        if (self.fr_ang_abs > 1.8 * pi and self.fr_ang_lst < 0.2 * pi):
            self.lap -= 1
        # encuetra si el cambio due de 2pi a 0
        if (self.fr_ang_abs < 0.2 * pi and self.fr_ang_lst > 1.8 * pi):
            self.lap += 1
        
        self.fr_ang_lap_lst = self.fr_ang
        self.fr_ang = 2 * pi * self.lap + self.fr_ang_abs

        self.fr_vel = 10. * (self.fr_ang - self.fr_ang_lap_lst)
    """


    def frResetCb(self, data):
        
        if (data.data == 1):
            self.fr_ang_des = 0


    """
    def frLecCb(self, data):
        
        self.fr_lec = data.data
        self.angCalc()
        self.pid()


    def frDesCb(self, data):

        self.fr_des = data.data
        self.angCalc()
        self.pid()
    """


    def frLecCb(self, data):

        self.times_lec += 1
        if (self.times_lec < 4):
            self.offset = data.data


        self.fr_lec = data.data
        self.angCalc()

        if (abs(self.fr_vel_des) < 0.1):
            # self.fr_ang_des = self.constrain(self.fr_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.fr_ang_des)
        else:
            self.fr_ang_des = self.fr_ang
            self.pid_vel()


    def frDesCb(self, data):

        self.fr_vel_des = data.data
        self.angCalc()

        if (abs(self.fr_vel_des) < 0.1):
            # self.fr_ang_des = self.constrain(self.fr_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.fr_ang_des)
        else:
            self.fr_ang_des = self.fr_ang
            self.pid_vel()


    def update(self):

        self.fr_save = abs(self.fr_out) + self.fr_save * 0.95 #MAXIMO DE 70 POR 35 (1 + 35 * 0.5 + ...)
        self.range_pos = 50 - self.fr_save / 40
        self.range_vel = 50 - self.fr_save / 40

        self.frOutPub.publish(self.fr_out)
        self.frAngPub.publish(self.fr_ang)
        self.frVelPub.publish(self.fr_vel)


    def spin(self):

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':

    """ main """
    fr_node = Fr_node()
    fr_node.spin() 
