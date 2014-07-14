#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

class Gripper_node:

    def __init__(self, node_name_override = 'gripper_node'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("gripper_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        self.gripper_out = 0
        self.gripper_ang = 0
        self.gripper_vel = 0

        self.gripperOutPub = rospy.Publisher("gripper_out", Int16)
        self.gripperAngPub = rospy.Publisher("gripper_ang", Float32)
        self.gripperVelPub = rospy.Publisher("gripper_vel", Float32)
        self.gripperResetSub = rospy.Subscriber("gripper_reset", Int16, self.gripperResetCb)
        self.gripperDesSub = rospy.Subscriber("gripper_des", Float32, self.gripperDesCb)


    def millis(self):

        return int(1000 * (self.rospy.get_time() - self.init_time))


    def constrain(self, x, min, max):

        if (x > max):
            return max
        if (x < min):
            return min
        return x


    def gripperResetCb(self, data):
        
        if (data.data == 1):
            self.gripper_out = 0


    def gripperDesCb(self, data):

        self.gripper_vel = data.data


    def update(self):

        if (abs(self.gripper_vel_des) < 0.1):
            # Mantiene posicion constante
            self.gripper_out = self.gripper_out
        else:
            # cambia segun la velocidad y restringe a rango
            self.gripper_out += self.gripper_vel * 200. / 3.1416
            self.gripper_out = self.constrain(self.gripper_out, -100, 100)

        # coversion a radianes
        self.gripper_ang = self.gripper_out * 3.1416 / 200.

        self.gripperOutPub.publish(int(self.gripper_out)
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