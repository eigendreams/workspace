#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from math import pi


class Base_node:

    def __init__(self, node_name_override = 'base_node'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("base_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        #TODO cambiar por parametros
        self.base_enc_ana = False
        self.base_enc_max = 1023
        self.base_ang_def = 0
        self.base_offset = 245

        # PID control parameters
        self.kp = 100.
        self.ki = 1.
        self.kd = 0.
        self.km = 0.
        self.umbral = 0.1
        self.range = 50. # Maximo pwm permitido
        self.kierr = 0.6
        self.kimax = 100.
        self.kisum = 0.
        self.error = 0.

        # topic variables
        self.base_lec = 0.
        self.base_ang = 0.
        self.base_des = 0.
        self.base_vel = 0.
        self.base_out = 0.

        # helper variables
        self.base_ang_tmp = 0.
        self.base_ang_rng = 0.
        self.base_lec_dst = 0.
        self.base_ang_lst = 0.
        self.base_ang_chg = 0.
        self.base_ang_abs = 0.
        self.base_ang_lap = 0.

        self.init_time = rospy.get_time()

        self.ticks = 0

        self.baseOutPub = rospy.Publisher("base_out", Int16)
        self.baseAngPub = rospy.Publisher("base_ang", Float32)
        self.baseVelPub = rospy.Publisher("base_vel", Float32)
        self.baseLecSub = rospy.Subscriber("base_lec", Int16, self.baseLecCb)
        #self.baseLecSub = rospy.Subscriber("base_debug", Int16, self.baseDbgCb)
        self.baseDesSub = rospy.Subscriber("base_des", Float32, self.baseDesCb)


    def map(self, x, in_min, in_max, out_min, out_max):

        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


    def millis(self):

        return int(1000 * (self.rospy.get_time() - self.init_time))


    def constrain(self, x, min, max):

        if (x > max):
            return max
        if (x < min):
            return min
        return x


    def pid(self):

        if (abs(self.base_des - self.base_ang) < self.umbral):
            self.error = 0.
        else:
            self.error = self.base_des - self.base_ang

        #print "error " + str(self.error)

        if (abs(self.base_des - self.base_ang) < self.kierr):
            if (abs(self.base_des - self.base_ang) < self.umbral):
                self.kisum = 0.;
            else:
                self.kisum += self.ki * self.error
                self.kisum = self.constrain(self.kisum, -self.kimax, self.kimax)
        else:
            self.kisum = 0.

        #print "kisum " + str(self.kisum)

        self.base_out = self.constrain(self.kp * self.error + self.kisum - self.kd * (self.base_ang - self.base_ang_lst) + self.km * self.base_des, -self.range, self.range)


    def angCalc(self):

        """MAP FIRST"""
        if self.base_lec < self.base_offset:
            self.base_ang_tmp = self.base_lec + self.base_enc_max + 1 - self.base_offset
        else:
            self.base_ang_tmp = self.base_lec - self.base_offset

        self.base_ang_tmp = self.map(self.base_ang_tmp, 0., self.base_enc_max, 2 * pi, 0.)
        self.base_ang_lst = self.base_ang_abs
        self.base_ang_abs = self.base_ang_tmp
        """MAP FIRST"""

        """LAP CALCULATE"""
        # encuentra si el cambio fue de 0 a 2pi
        if (self.base_ang_abs > 1.8 * pi and self.base_ang_lst < 0.2 * pi):
            self.base_ang_lap -= 1
        # encuentra si el cambio fue de 2pi a 0
        if (self.base_ang_abs < 0.2 * pi and self.base_ang_lst > 1.8 * pi):
            self.base_ang_lap += 1
        """LAP CALCULATE"""

        self.base_ang_lap_lst = self.base_ang
        self.base_ang = 2 * pi * self.base_ang_lap + self.base_ang_abs

        if (self.ticks < 10):
            if (self.base_ang < -3):
                self.base_ang += 2 * pi

        """MAP VEL OUT"""
        self.base_vel = self.base_ang - self.base_ang_lap_lst
        """MAP VEL OUT"""


    def desCalc(self, data):

        self.base_des = self.constrain(self.base_des, )


    def baseDbgCb(self, data):

        self.base_out = data.data

    def baseLecCb(self, data):
        
        self.base_lec = data.data
        self.angCalc()
        self.pid()
        self.ticks += 1


    def baseDesCb(self, data):

        self.base_des = data.data
        self.angCalc()
        self.pid()


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
    base_node = Base_node()
    base_node.spin()
    
