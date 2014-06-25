#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from math import pi


class Gripper_node:

    def __init__(self, node_name_override = 'gripper_node'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("gripper_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        #TODO cambiar por parametros
        self.gripper_enc_ana = False
        self.gripper_enc_max = 1006
        self.gripper_ang_def = 0
        self.gripper_offset = 745

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
        self.gripper_lec = 0
        self.gripper_ang = 0
        self.gripper_des = 0
        self.gripper_vel = 0
        self.gripper_out = 0

        # helper variables
        self.gripper_ang_tmp = 0
        self.gripper_ang_rng = 0
        self.gripper_lec_dst = 0
        self.gripper_ang_lst = 0
        self.gripper_ang_chg = 0
        self.gripper_ang_abs = 0
        self.gripper_ang_lap = 0

        self.init_time = rospy.get_time()

        self.gripperOutPub = rospy.Publisher("gripper_out", Int16)
        self.gripperAngPub = rospy.Publisher("gripper_ang", Float32)
        self.gripperVelPub = rospy.Publisher("gripper_vel", Float32)
        self.gripperLecSub = rospy.Subscriber("gripper_lec", Int16, self.gripperLecCb)
        self.gripperDesSub = rospy.Subscriber("gripper_des", Float32, self.gripperDesCb)


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

        if (abs(self.gripper_des - self.gripper_ang) < self.umbral):
           self.error = 0.
        else:
            self.error = self.gripper_des - self.gripper_ang

        if (abs(self.gripper_des - self.gripper_ang) < self.kierr):
            if (abs(self.gripper_des - self.gripper_ang) < self.umbral):
                self.kisum = 0.;
            else:
                self.kisum += self.ki * self.error
                self.kisum = self.constrain(self.kisum, -self.kimax, self.kimax)
        else:
            self.kisum = 0.

        self.gripper_out = self.constrain(self.kp * self.error + self.kisum - self.kd * (self.gripper_ang - self.gripper_ang_lst) + self.km * self.gripper_des, -self.range, self.range)


    def angCalc(self):

        """MAP FIRST"""
        if self.gripper_lec < self.gripper_offset:
            self.gripper_ang_tmp = self.gripper_lec + self.gripper_enc_max + 1 - self.gripper_offset
        else:
            self.gripper_ang_tmp = self.gripper_lec - self.gripper_offset

        self.gripper_ang_tmp = self.map(self.gripper_ang_tmp, 0., self.gripper_enc_max, 2 * pi, 0.)
        self.gripper_ang_lst = self.gripper_ang_abs
        self.gripper_ang_abs = self.gripper_ang_tmp
        """MAP FIRST"""

        """LAP CALCULATE"""
        # encuentra si el cambio fue de 0 a 2pi
        if (self.gripper_ang_abs > 1.8 * pi and self.gripper_ang_lst < 0.2 * pi):
            self.gripper_ang_lap -= 1
        # encuentra si el cambio fue de 2pi a 0
        if (self.gripper_ang_abs < 0.2 * pi and self.gripper_ang_lst > 1.8 * pi):
            self.gripper_ang_lap += 1
        """LAP CALCULATE"""

        self.gripper_ang_lap_lst = self.gripper_ang
        self.gripper_ang = 2 * pi * self.gripper_ang_lap + self.gripper_ang_abs

        """MAP VEL OUT"""
        self.gripper_vel = self.gripper_ang - self.gripper_ang_lap_lst
        """MAP VEL OUT"""


    def desCalc(self, data):

        self.gripper_des = self.constrain(self.gripper_des, )


    def gripperLecCb(self, data):
        
        self.gripper_lec = data.data
        self.angCalc()
        self.pid()


    def gripperDesCb(self, data):

        self.gripper_des = data.data
        self.angCalc()
        self.pid()


    def update(self):

        self.gripperOutPub.publish(self.gripper_out)
        self.gripperAngPub.publish(self.gripper_ang)
        self.gripperVelPub.publish(self.gripper_vel)


    def spin(self):

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':

    """ main """
    gripper_node = Gripper_node()
    gripper_node.spin()
    