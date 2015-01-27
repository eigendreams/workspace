#!/usr/bin/env python
# -*- coding: utf8 -*-
################################################################################
from math import *
################################################################################
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
################################################################################
from volti.cfg import PIDConfig
################################################################################
from ino_mod import *
from kfilter import *
from limit   import *
from pid     import *
from encoder import *
################################################################################
from numpy import *
from numpy.linalg import inv 
################################################################################
#
class Motor_log:
    #
    def __init__(self, node_name_default = 'motor_log'):
        #
        rospy.init_node(node_name_default)
        self.nodename = rospy.get_name()
        rospy.loginfo("motor_log starting with name %s", self.nodename)
        self.rate = float(rospy.get_param("param_global_rate", '100'))
        #
        self.times = 0
        #
        # filtro de entradas del encoder
        self.kf_settings = {'Q' : 10, 'R' : 10, 'P0' : 10, 'rate' : 10}
        self.filter_e1  = Kfilter(self.kf_settings)
        self.enc_settings = {'offset' : -1}
        self.enc_1 = Encoder(self.enc_settings)
        #
        # Asociaciones con publicadores y suscriptores
        self.m1 = rospy.Publisher("m1", Int16)  # salida al motor 1
        self.al = rospy.Publisher("al", Int16)
        self.e1ang = rospy.Publisher("e1ang", Float32)  #
        self.e1vel = rospy.Publisher("e1vel", Float32)  #
        self.e1accel = rospy.Publisher("e1accel", Float32)  #
        #
        self.val = 5
        #
        self.e1 = rospy.Subscriber("e1", Int16, self.e1cb)  # entrada del encoder 1
        #
    def e1cb(self, data):
        #
        self.value_m1 = data.data
        self.proc_m1 = self.enc_1.compute(self.value_m1)
        self.X_m1 = self.filter_e1.compute(self.proc_m1)
        #
        self.angle_m1 = self.X_m1[0, 0]
        self.speed_m1 = self.X_m1[1, 0]
        self.accel_m1 = self.X_m1[2, 0]
        #
        self.e1ang.publish(self.angle_m1)
        self.e1vel.publish(self.speed_m1)
        self.e1accel.publish(self.accel_m1)
        #
        print(str(self.times * 0.01) + "," + str(100 * self.speed_m1))
        #
        #
    def update(self):
        #
        self.times = self.times + 1
        #
        if ((self.times % 500) == 0):
            self.val =  -self.val
        #
        self.m1.publish(self.val + 5)
        #
        #
    def spin(self):
        #
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
            if (self.times % 100 == 0):
                self.al.publish(1)
            #
            #
if __name__ == '__main__':
    #
    """ main """
    motor_log = Motor_log()
    motor_log.spin() 
