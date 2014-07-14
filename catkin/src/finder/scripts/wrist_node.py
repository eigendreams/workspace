#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import pi

class Wrist_node:

    def __init__(self, node_name_override = 'wrist_node'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("wrist_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        self.wristOutPub = rospy.Publisher("wrist_out", Int16)
        self.wristAngPub = rospy.Publisher("wrist_ang", Float32)
        self.wristVelPub = rospy.Publisher("wrist_vel", Float32)

        #TODO cambiar por parametros
        self.wrist_enc_ana = False
        self.wrist_enc_max = 1008
        self.wrist_offset = 590

        """COSTANTES"""
        """kevin test"""
        # PID control parameters
        self.kp_pos = 33
        self.ki_pos = 1
        self.kd_pos = 0
        self.km_pos = 0
        self.umbral_pos = 0.05
        self.range_pos = 40 # Maximo pwm permitido
        self.kierr_pos = 0.6
        self.kimax_pos = 20
        self.kisum_pos = 0
        self.error_pos = 0

        self.kp_vel = 25
        self.ki_vel = 1
        self.kd_vel = 0
        self.km_vel = 0
        self.umbral_vel = 0.05
        self.range_vel = 40 # Maximo pwm permitido
        self.kierr_vel = 0.6
        self.kimax_vel = 20
        self.kisum_vel = 0
        self.error_vel = 0

        # topic variables
        self.wrist_lec = 0
        self.wrist_des = 0
        self.wrist_ang = 0
        self.wrist_ang_des = 0
        self.wrist_vel = 0
        self.wrist_vel_des = 0
        self.wrist_out = 0

        # helper variables
        self.wrist_ang_tmp = 0
        self.wrist_ang_rng = 0
        self.wrist_lec_dst = 0
        self.wrist_ang_lst = 0
        self.wrist_ang_chg = 0
        self.wrist_ang_abs = 0
        self.wrist_ang_lap = 0

        self.wrist_offset_internal = 0.
        self.wrist_ang_tmp_internal = 0      
        self.wrist_ang_lst_internal = 0
        self.wrist_ang_abs_internal = 0

        self.times = 0
        self.init_time = rospy.get_time()

        self.wristResetSub = rospy.Subscriber("wrist_reset", Int16, self.wristResetCb)
        self.wristLecSub = rospy.Subscriber("wrist_lec", Int16, self.wristLecCb)
        self.wristDesSub = rospy.Subscriber("wrist_des", Float32, self.wristDesCb)
        self.offsetSub = rospy.Subscriber("offset", Int16, self.offsetCb)


    def offsetCb(self, data):

        if (data.data == 1):

            self.wrist_offset  = self.wrist_lec
            
            self.wrist_ang_tmp = 0
            self.wrist_ang_lst = 0
            self.wrist_ang_abs = 0

            self.wrist_ang = 0
            self.wrist_ang_lap =  0
            self.wrist_ang_lap_lst = 0


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

        if (abs(self.wrist_des - self.wrist_ang) < self.umbral_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.wrist_des - self.wrist_ang

        if (abs(self.wrist_des - self.wrist_ang) < self.kierr_pos):
            if (abs(self.wrist_des - self.wrist_ang) < self.umbral_pos):
                #pass
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
        else:
            self.kisum_pos = 0.

        self.wrist_out = -self.constrain(self.kp_pos * self.error_pos + self.kisum_pos - self.kd_pos * (self.wrist_ang - self.wrist_ang_lst) + self.km_pos * self.wrist_des, -self.range_pos, self.range_pos)
    """


    def pid_pos(self):

        if (abs(self.wrist_ang_des - self.wrist_ang) < self.umbral_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.wrist_ang_des - self.wrist_ang
            #print "error " + str(self.error)

        if (abs(self.wrist_ang_des - self.wrist_ang) < self.kierr_pos):
            if (abs(self.wrist_ang_des - self.wrist_ang) < self.umbral_pos):
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_pos = 0.

        self.wrist_out = self.constrain(self.kp_pos * self.error_pos + self.kisum_pos - self.kd_pos * (self.wrist_ang - self.wrist_ang_lst) + self.km_pos * self.wrist_ang_des, -self.range_pos, self.range_pos)


    def pid_vel(self):

        if (abs(self.wrist_vel_des - self.wrist_vel) < self.umbral_vel):
           self.error_vel = 0.
        else:
            self.error_vel = self.wrist_vel_des - self.wrist_vel + 0.
            #print "error " + str(self.error)

        if (abs(self.wrist_vel_des - self.wrist_vel) < self.kierr_vel):
            if (abs(self.wrist_vel_des - self.wrist_vel) < self.umbral_vel):
                self.kisum_vel = 0.;
            else:
                self.kisum_vel += self.ki_vel * self.error_vel
                self.kisum_vel = self.constrain(self.kisum_vel, -self.kimax_vel, self.kimax_vel)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_vel = 0.

        self.wrist_out = self.constrain(self.kp_vel * self.error_vel + self.kisum_vel - self.kd_pos * (self.wrist_ang - self.wrist_ang_lst) + self.km_vel * self.wrist_vel_des, -self.range_vel, self.range_vel)


    def angCalc(self):

        self.times += 1

        self.wrist_offset_internal = self.map(self.wrist_offset, 0., self.wrist_enc_max, 0., 2 * pi)
        self.wrist_ang_tmp_internal = self.map(self.wrist_lec, 0., self.wrist_enc_max, 0, 2 * pi)
        self.wrist_ang_lst_internal = self.wrist_ang_abs_internal
        self.wrist_ang_abs_internal = self.wrist_ang_tmp_internal

        if (self.times > 4):
            # encuentra si el cambio fue de 0 a 2pi
            if (self.wrist_ang_abs_internal > 1.7 * pi and self.wrist_ang_lst_internal < 0.3 * pi):
                self.wrist_ang_lap -= 1
            # encuentra si el cambio fue de 2pi a 0
            if (self.wrist_ang_abs_internal < 0.3 * pi and self.wrist_ang_lst_internal > 1.7 * pi):
                self.wrist_ang_lap += 1

        self.wrist_ang_lap_lst = self.wrist_ang
        self.wrist_ang = 2 * pi * self.wrist_ang_lap + self.wrist_ang_abs_internal - self.wrist_offset_internal 
        self.wrist_vel = 10 * (self.wrist_ang - self.wrist_ang_lap_lst)


    def wristResetCb(self, data):
        
        if (data.data == 1):
            self.wrist_ang_des = 0


    """
    def wristLecCb(self, data):
        
        self.wrist_lec = data.data
        self.angCalc()
        self.pid()


    def wristDesCb(self, data):

        self.wrist_des = data.data
        self.angCalc()
        self.pid()
    """


    def wristLecCb(self, data):

        self.wrist_lec = data.data
        self.angCalc()

        if (abs(self.wrist_vel_des) < 0.1):
            # self.wrist_ang_des = self.constrain(self.wrist_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.wrist_ang_des)
        else:
            self.wrist_ang_des = self.wrist_ang
            self.pid_vel()


    def wristDesCb(self, data):

        self.wrist_vel_des = data.data
        self.angCalc()

        if (abs(self.wrist_vel_des) < 0.1):
            # self.wrist_ang_des = self.constrain(self.wrist_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.wrist_ang_des)
        else:
            self.wrist_ang_des = self.wrist_ang
            self.pid_vel()


    def update(self):

        self.wristOutPub.publish(-self.wrist_out)
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
    
