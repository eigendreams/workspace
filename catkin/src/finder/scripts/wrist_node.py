#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from math import pi


class Wrist_node:

    def __init__(self, node_name_override = 'wrist_node'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("wrist_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        #TODO cambiar por parametros
        self.wrist_enc_ana = False
        self.wrist_enc_max = 1006
        self.wrist_ang_def = 0
        self.wrist_offset = 800

        # PID control parameters
        self.kp = 25   
        self.ki = 5
        self.kd = 0
        self.km = 0
        self.umbral = 0.1
        self.range = 40 # Maximo pwm permitido
        self.kierr = 0.6
        self.kimax = 40
        self.kisum = 0
        self.error = 0

        # topic variables
        self.wrist_lec = 0
        self.wrist_ang = 0
        self.wrist_des = 0
        self.wrist_vel = 0
        self.wrist_out = 0

        # helper variables
        self.wrist_ang_tmp = 0
        self.wrist_ang_rng = 0
        self.wrist_lec_dst = 0
        self.wrist_ang_lst = 0
        self.wrist_ang_chg = 0
        self.wrist_ang_abs = 0
        self.wrist_ang_lap = 0

        self.init_time = rospy.get_time()

        self.wristOutPub = rospy.Publisher("wrist_out", Int16)
        self.wristAngPub = rospy.Publisher("wrist_ang", Float32)
        self.wristVelPub = rospy.Publisher("wrist_vel", Float32)
        self.wristLecSub = rospy.Subscriber("wrist_lec", Int16, self.wristLecCb)
        self.wristDesSub = rospy.Subscriber("wrist_des", Float32, self.wristDesCb)
        self.offsetSub = rospy.Subscriber("offset", Int16, self.offsetCb)

    def offsetCb(self, data):

        if (data.data):
            self.wrist_offset  = self.wrist_lec
            
            self.wrist_ang_tmp = 0
            self.wrist_ang_lst = 0
            self.wrist_ang_abs = 0

            self.wrist_ang = 0
            self.wrist_ang_lap =  0
            self.wrist_ang_lap_lst = 0


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

        if (abs(self.wrist_des - self.wrist_ang) < self.umbral):
           self.error = 0.
        else:
            self.error = self.wrist_des - self.wrist_ang

        if (abs(self.wrist_des - self.wrist_ang) < self.kierr):
            if (abs(self.wrist_des - self.wrist_ang) < self.umbral):
                pass
                #self.kisum = 0.;
            else:
                self.kisum += self.ki * self.error
                self.kisum = self.constrain(self.kisum, -self.kimax, self.kimax)
        else:
            self.kisum = 0.

        self.wrist_out = self.constrain(self.kp * self.error + self.kisum - self.kd * (self.wrist_ang - self.wrist_ang_lst) + self.km * self.wrist_des, -self.range, self.range)


    def angCalc(self):

        """MAP FIRST"""
        if self.wrist_lec < self.wrist_offset:
            self.wrist_ang_tmp = self.wrist_lec + self.wrist_enc_max + 1 - self.wrist_offset
        else:
            self.wrist_ang_tmp = self.wrist_lec - self.wrist_offset

        self.wrist_ang_tmp = self.map(self.wrist_ang_tmp, 0., self.wrist_enc_max, 0, 2* pi)
        self.wrist_ang_lst = self.wrist_ang_abs
        self.wrist_ang_abs = self.wrist_ang_tmp
        """MAP FIRST"""

        """LAP CALCULATE"""
        # encuentra si el cambio fue de 0 a 2pi
        if (self.wrist_ang_abs > 1.8 * pi and self.wrist_ang_lst < 0.2 * pi):
            self.wrist_ang_lap -= 1
        # encuentra si el cambio fue de 2pi a 0
        if (self.wrist_ang_abs < 0.2 * pi and self.wrist_ang_lst > 1.8 * pi):
            self.wrist_ang_lap += 1
        """LAP CALCULATE"""

        self.wrist_ang_lap_lst = self.wrist_ang
        self.wrist_ang = 2 * pi * self.wrist_ang_lap + self.wrist_ang_abs

        """MAP VEL OUT"""
        self.wrist_vel = self.wrist_ang - self.wrist_ang_lap_lst
        """MAP VEL OUT"""


    def desCalc(self, data):

        self.wrist_des = self.constrain(self.wrist_des, )


    def wristLecCb(self, data):
        
        self.wrist_lec = data.data
        self.angCalc()
        self.pid()


    def wristDesCb(self, data):

        self.wrist_des = data.data
        self.angCalc()
        self.pid()


    def update(self):

        self.wristOutPub.publish(self.wrist_out)
        self.wristAngPub.publish(self.wrist_ang)
        self.wristVelPub.publish(self.wrist_vel)


    def spin(self):

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':

    """ main """
    wrist_node = Wrist_node()
    wrist_node.spin()
    
