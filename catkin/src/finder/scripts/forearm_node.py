#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from math import pi


class Forearm_node:

    def __init__(self, node_name_override = 'forearm_node'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("forearm_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        #TODO cambiar por parametros
        self.forearm_enc_ana = False
        self.forearm_enc_max = 1023
        self.forearm_ang_def = 0
        self.forearm_offset = 745

        # PID control parameters
        self.kp = 100
        self.ki = 10
        self.kd = 0
        self.km = 0
        self.umbral = 0.1
        self.range = 100 # Maximo pwm permitido
        self.kierr = 1.2
        self.kimax = 100
        self.kisum = 0
        self.error = 0

        # topic variables
        self.forearm_lec = 0
        self.forearm_ang = 0
        self.forearm_des = 0
        self.forearm_vel = 0
        self.forearm_out = 0

        # helper variables
        self.forearm_ang_tmp = 0
        self.forearm_ang_rng = 0
        self.forearm_lec_dst = 0
        self.forearm_ang_lst = 0
        self.forearm_ang_chg = 0
        self.forearm_ang_abs = 0
        self.forearm_ang_lap = 0

        self.init_time = rospy.get_time()

        self.forearmOutPub = rospy.Publisher("forearm_out", Int16)
        self.forearmAngPub = rospy.Publisher("forearm_ang", Float32)
        self.forearmVelPub = rospy.Publisher("forearm_vel", Float32)
        self.forearmLecSub = rospy.Subscriber("forearm_lec", Int16, self.forearmLecCb)
        self.forearmDesSub = rospy.Subscriber("forearm_des", Float32, self.forearmDesCb)


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

        if (abs(self.forearm_des - self.forearm_ang) < self.umbral):
           self.error = 0.
        else:
            self.error = self.forearm_des - self.forearm_ang

        if (abs(self.forearm_des - self.forearm_ang) < self.kierr):
            if (abs(self.forearm_des - self.forearm_ang) < self.umbral):
                self.kisum = 0.;
            else:
                self.kisum += self.ki * self.error
                self.kisum = self.constrain(self.kisum, -self.kimax, self.kimax)
        else:
            self.kisum = 0.

        self.forearm_out = self.constrain(self.kp * self.error + self.kisum - self.kd * (self.forearm_ang - self.forearm_ang_lst) + self.km * self.forearm_des, -self.range, self.range)


    def angCalc(self):

        """MAP FIRST"""
        if self.forearm_lec < self.forearm_offset:
            self.forearm_ang_tmp = self.forearm_lec + self.forearm_enc_max + 1 - self.forearm_offset
        else:
            self.forearm_ang_tmp = self.forearm_lec - self.forearm_offset

        self.forearm_ang_tmp = self.map(self.forearm_ang_tmp, 0., self.forearm_enc_max, 2 * pi, 0.)
        self.forearm_ang_lst = self.forearm_ang_abs
        self.forearm_ang_abs = self.forearm_ang_tmp
        """MAP FIRST"""

        """LAP CALCULATE"""
        # encuentra si el cambio fue de 0 a 2pi
        if (self.forearm_ang_abs > 1.8 * pi and self.forearm_ang_lst < 0.2 * pi):
            self.forearm_ang_lap -= 1
        # encuentra si el cambio fue de 2pi a 0
        if (self.forearm_ang_abs < 0.2 * pi and self.forearm_ang_lst > 1.8 * pi):
            self.forearm_ang_lap += 1
        """LAP CALCULATE"""

        self.forearm_ang_lap_lst = self.forearm_ang
        self.forearm_ang = 2 * pi * self.forearm_ang_lap + self.forearm_ang_abs

        """MAP VEL OUT"""
        self.forearm_vel = self.forearm_ang - self.forearm_ang_lap_lst
        """MAP VEL OUT"""


    def desCalc(self, data):

        self.forearm_des = self.constrain(self.forearm_des, )


    def forearmLecCb(self, data):
        
        self.forearm_lec = data.data
        self.angCalc()
        self.pid()


    def forearmDesCb(self, data):

        self.forearm_des = data.data
        self.angCalc()
        self.pid()


    def update(self):

        self.forearmOutPub.publish(self.forearm_out)
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
    
