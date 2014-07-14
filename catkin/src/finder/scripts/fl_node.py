#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

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
        self.umbral_pos = 1
        self.range_pos = 35 # Maximo pwm permitido
        self.kierr_pos = 2
        self.kimax_pos = 35
        self.kisum_pos = 0
        self.error_pos = 0

        self.kp_vel = 5
        self.ki_vel = 2
        self.kd_vel = 0
        self.km_vel = 0
        self.umbral_vel = 0.5
        self.range_vel = 35 # Maximo pwm permitido
        self.kierr_vel = 1
        self.kimax_vel = 35
        self.kisum_vel = 0
        self.error_vel = 0

        # topic variables
        self.fl_lec = 0
        self.fl_des = 0
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
        self.fl_ang_lap = 0

        self.fl_offset_internal = 0.
        self.fl_ang_tmp_internal = 0      
        self.fl_ang_lst_internal = 0
        self.fl_ang_abs_internal = 0

        self.times = 0
        self.init_time = rospy.get_time()

        self.flResetSub = rospy.Subscriber("fl_reset", Int16, self.flResetCb)
        self.flLecSub = rospy.Subscriber("fl_lec", Int16, self.flLecCb)
        self.flDesSub = rospy.Subscriber("fl_des", Float32, self.flDesCb)
        self.offsetSub = rospy.Subscriber("offset", Int16, self.offsetCb)


    def offsetCb(self, data):

        if (data.data == 1):

            self.fl_offset  = self.fl_lec
            
            self.fl_ang_tmp = 0
            self.fl_ang_lst = 0
            self.fl_ang_abs = 0

            self.fl_ang = 0
            self.fl_ang_lap =  0
            self.fl_ang_lap_lst = 0


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

        if (abs(self.fl_des - self.fl_ang) < self.umbral_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.fl_des - self.fl_ang

        if (abs(self.fl_des - self.fl_ang) < self.kierr_pos):
            if (abs(self.fl_des - self.fl_ang) < self.umbral_pos):
                #pass
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
        else:
            self.kisum_pos = 0.

        self.fl_out = -self.constrain(self.kp_pos * self.error_pos + self.kisum_pos - self.kd_pos * (self.fl_ang - self.fl_ang_lst) + self.km_pos * self.fl_des, -self.range_pos, self.range_pos)
    """


    def pid_pos(self):

        if (abs(self.fl_ang_des - self.fl_ang) < self.umbral_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.fl_ang_des - self.fl_ang
            #print "error " + str(self.error)

        if (abs(self.fl_ang_des - self.fl_ang) < self.kierr_pos):
            if (abs(self.fl_ang_des - self.fl_ang) < self.umbral_pos):
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_pos = 0.

        self.fl_out = self.constrain(self.kp_pos * self.error_pos + self.kisum_pos - self.kd_pos * (self.fl_ang - self.fl_ang_lst) + self.km_pos * self.fl_ang_des, -self.range_pos, self.range_pos)


    def pid_vel(self):

        if (abs(self.fl_vel_des - self.fl_vel) < self.umbral_vel):
           self.error_vel = 0.
        else:
            self.error_vel = self.fl_vel_des - self.fl_vel + 0.
            #print "error " + str(self.error)

        if (abs(self.fl_vel_des - self.fl_vel) < self.kierr_vel):
            if (abs(self.fl_vel_des - self.fl_vel) < self.umbral_vel):
                self.kisum_vel = 0.;
            else:
                self.kisum_vel += self.ki_vel * self.error_vel
                self.kisum_vel = self.constrain(self.kisum_vel, -self.kimax_vel, self.kimax_vel)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_vel = 0.

        self.fl_out = self.constrain(self.kp_vel * self.error_vel + self.kisum_vel - self.kd_pos * (self.fl_ang - self.fl_ang_lst) + self.km_vel * self.fl_vel_des, -self.range_vel, self.range_vel)


    def angCalc(self):

        self.times += 1

        self.fl_offset_internal = self.map(self.fl_offset, 0., self.fl_enc_max, 2 * pi, 0)
        self.fl_ang_tmp_internal = self.map(self.fl_lec, 0., self.fl_enc_max, 2 * pi, 0)
        self.fl_ang_lst_internal = self.fl_ang_abs_internal
        self.fl_ang_abs_internal = self.fl_ang_tmp_internal

        if (self.times > 4):
            # encuentra si el cambio fue de 0 a 2pi
            if (self.fl_ang_abs_internal > 1.7 * pi and self.fl_ang_lst_internal < 0.3 * pi):
                self.fl_ang_lap -= 1
            # encuentra si el cambio fue de 2pi a 0
            if (self.fl_ang_abs_internal < 0.3 * pi and self.fl_ang_lst_internal > 1.7 * pi):
                self.fl_ang_lap += 1

        self.fl_ang_lap_lst = self.fl_ang
        self.fl_ang = 2 * pi * self.fl_ang_lap + self.fl_ang_abs_internal - self.fl_offset_internal 
        self.fl_vel = 10 * (self.fl_ang - self.fl_ang_lap_lst)


    """
    def angCalc(self):

        self.fl_ang_tmp = self.fl_lec        
        self.fl_ang_tmp = self.map(self.fl_ang_tmp, 0., 1023., 2*pi, 0.)
        self.fl_ang_lst = self.fl_ang_abs
        self.fl_ang_abs = self.fl_ang_tmp

        # encuentra si el cambio fue de 0 a 2pi
        if (self.fl_ang_abs > 1.8 * pi and self.fl_ang_lst < 0.2 * pi):
            self.lap -= 1
        # encuetra si el cambio due de 2pi a 0
        if (self.fl_ang_abs < 0.2 * pi and self.fl_ang_lst > 1.8 * pi):
            self.lap += 1
        
        self.fl_ang_lap_lst = self.fl_ang
        self.fl_ang = 2 * pi * self.lap + self.fl_ang_abs

        self.fl_vel = 10. * (self.fl_ang - self.fl_ang_lap_lst)
    """


    def flResetCb(self, data):
        
        if (data.data == 1):
            self.fl_ang_des = 0


    """
    def flLecCb(self, data):
        
        self.fl_lec = data.data
        self.angCalc()
        self.pid()


    def flDesCb(self, data):

        self.fl_des = data.data
        self.angCalc()
        self.pid()
    """


    def flLecCb(self, data):

        self.fl_lec = data.data
        self.angCalc()

        if (abs(self.fl_vel_des) < 0.1):
            # self.fl_ang_des = self.constrain(self.fl_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.fl_ang_des)
        else:
            self.fl_ang_des = self.fl_ang
            self.pid_vel()


    def flDesCb(self, data):

        self.fl_vel_des = data.data
        self.angCalc()

        if (abs(self.fl_vel_des) < 0.1):
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
