#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from math import pi


class Arm_node:

    def __init__(self, node_name_override = 'arm_node'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("arm_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        #TODO cambiar por parametros
        self.arm_enc_ana = False
        self.arm_enc_max = 1023
        self.arm_ang_def = 0
        self.arm_offset = 745

        # PID control parameters
        self.kp = 100
        self.ki = 1
        self.kd = 0
        self.km = 0
        self.umbral = 0.1
        self.range = 100 # Maximo pwm permitido
        self.kierr = 1.2
        self.kimax = 100
        self.kisum = 0
        self.error = 0

        # topic variables
        self.arm_lec = 0
        self.arm_ang = 0
        self.arm_des = 0
        self.arm_vel = 0
        self.arm_out = 0

        # helper variables
        self.arm_ang_tmp = 0
        self.arm_ang_rng = 0
        self.arm_lec_dst = 0
        self.arm_ang_lst = 0
        self.arm_ang_chg = 0
        self.arm_ang_abs = 0
        self.arm_ang_lap = 0

        self.init_time = rospy.get_time()

        self.armOutPub = rospy.Publisher("arm_out", Int16)
        self.armAngPub = rospy.Publisher("arm_ang", Float32)
        self.armVelPub = rospy.Publisher("arm_vel", Float32)
        self.armLecSub = rospy.Subscriber("arm_lec", Int16, self.armLecCb)
        self.armDesSub = rospy.Subscriber("arm_des", Float32, self.armDesCb)
        self.offsetSub = rospy.Subscriber("offset", Int16, self.offsetCb)

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

        if (abs(self.arm_des - self.arm_ang) < self.umbral):
           self.error = 0.
        else:
            self.error = self.arm_des - self.arm_ang

        if (abs(self.arm_des - self.arm_ang) < self.kierr):
            if (abs(self.arm_des - self.arm_ang) < self.umbral):
                self.kisum = 0.;
            else:
                self.kisum += self.ki * self.error
                self.kisum = self.constrain(self.kisum, -self.kimax, self.kimax)
        else:
            self.kisum = 0.

        self.arm_out = self.constrain(self.kp * self.error + self.kisum - self.kd * (self.arm_ang - self.arm_ang_lst) + self.km * self.arm_des, -self.range, self.range)

    def offsetCb(self):

        self.arm_offset  = self.arm_lec
        
        self.arm_ang_tmp = 0
        self.arm_ang_lst = 0
        self.arm_ang_abs = 0

        self.arm_ang = 0
        self.arm_ang_lap =  0
        self.arm_ang_lap_lst = 0


    def angCalc(self):

        """MAP FIRST"""
        if self.arm_lec < self.arm_offset:
            self.arm_ang_tmp = self.arm_lec + self.arm_enc_max + 1 - self.arm_offset
        else:
            self.arm_ang_tmp = self.arm_lec - self.arm_offset

        self.arm_ang_tmp = self.map(self.arm_ang_tmp, 0., self.arm_enc_max, 2 * pi, 0.)
        self.arm_ang_lst = self.arm_ang_abs
        self.arm_ang_abs = self.arm_ang_tmp
        """MAP FIRST"""

        """LAP CALCULATE"""
        # encuentra si el cambio fue de 0 a 2pi
        if (self.arm_ang_abs > 1.8 * pi and self.arm_ang_lst < 0.2 * pi):
            self.arm_ang_lap -= 1
        # encuentra si el cambio fue de 2pi a 0
        if (self.arm_ang_abs < 0.2 * pi and self.arm_ang_lst > 1.8 * pi):
            self.arm_ang_lap += 1
        """LAP CALCULATE"""

        self.arm_ang_lap_lst = self.arm_ang
        self.arm_ang = 2 * pi * self.arm_ang_lap + self.arm_ang_abs

        """MAP VEL OUT"""
        self.arm_vel = self.arm_ang - self.arm_ang_lap_lst
        """MAP VEL OUT"""


    def desCalc(self, data):

        self.arm_des = self.constrain(self.arm_des, )


    def armLecCb(self, data):
        
        self.arm_lec = data.data
        self.angCalc()
        self.pid()


    def armDesCb(self, data):

        self.arm_des = data.data
        #self.angCalc()
        self.pid()


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
    arm_node = Arm_node()
    arm_node.spin()
    
