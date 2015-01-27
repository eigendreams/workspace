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
from kfilter import Kfilter as Kfilter1
from kfilter_old import Kfilter as Kfilter2
from kfilter_new import Kfilter as Kfilter3
from kfilter_trans import Kfilter as Kfilter4
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
        self.rate = float(rospy.get_param("param_global_rate", '10'))
        #
        self.times = 0
        #
        # filtro de entradas del encoder
        self.kf_settings = {'Q' : 10, 'R' : 10, 'P0' : 10, 'rate' : 10}
        self.filter_e1k1  = Kfilter1(self.kf_settings)
        ###
        ###
        ###
        self.filter_e1k2  = Kfilter2(self.kf_settings)
        self.filter_e1k3  = Kfilter3(self.kf_settings)
        self.filter_e1k4  = Kfilter4(self.kf_settings)
        ###
        ###
        ###
        self.enc_settings = {'offset' : -1}
        self.enc_1 = Encoder(self.enc_settings)
        #
        # Asociaciones con publicadores y suscriptores
        self.m1 = rospy.Publisher("m1pass", Int16)  # salida al motor 1
        self.al = rospy.Publisher("al", Int16)
        self.e1ang = rospy.Publisher("e1ang", Float32)  #
        self.e1velk1 = rospy.Publisher("e1vel_k1", Float32)  #
        ###
        ###
        ###
        self.e1velk2 = rospy.Publisher("e1vel_k2", Float32)  #
        self.e1velk3 = rospy.Publisher("e1vel_k3", Float32)  #
        self.e1velk4 = rospy.Publisher("e1vel_k4", Float32)  #
        ###
        ###
        ###
        self.val = 4
        #
        self.e1 = rospy.Subscriber("e1", Int16, self.e1cb)  # entrada del encoder 1
        #
    def e1cb(self, data):
        #
        self.value_m1 = data.data
        self.proc_m1 = self.enc_1.compute(self.value_m1)
        self.X_m1 = self.filter_e1k1.compute(self.proc_m1)
        #
        self.angle_m1 = self.X_m1[0, 0]
        self.speed_m1 = self.X_m1[1, 0]
        self.accel_m1 = self.X_m1[2, 0]
        #
        self.e1ang.publish(self.angle_m1)
        self.e1velk1.publish(self.speed_m1)
        ##
        ##
        ##
        self.e1velk2.publish((self.filter_e1k2.compute(self.proc_m1))[1, 0])
        self.e1velk3.publish((self.filter_e1k3.compute(self.proc_m1))[1, 0])
        self.e1velk4.publish((self.filter_e1k4.compute(self.proc_m1))[1, 0])
        ##
        ##
        ##
        print(str(self.times * 0.01) + "," + str(100 * self.speed_m1))
        #
        #
    def update(self):
        #
        self.times = self.times + 1
        #
        if ((self.times % 10) == 0):
            self.val =  self.val
            self.al.publish(1)
        #
        self.m1.publish(self.val)                
        #
        #
    def spin(self):
        #
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
            #
            #
if __name__ == '__main__':
    #
    """ main """
    motor_log = Motor_log()
    motor_log.spin() 
