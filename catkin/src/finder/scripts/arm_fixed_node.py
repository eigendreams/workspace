#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from math import pi


class Arm_fixed_node:

    def __init__(self, node_name_override = 'arm_fixed_node'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("arm_fixed_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        self.init_time = rospy.get_time()

        self.arm_des = 0
        self.forearm_des = 0
        self.wrist_des = 0
        self.palm_des = 0
        self.gripper_des = 0

        self.arm_pos = 0

        self.values = [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]

        self.armDesPub = rospy.Publisher("arm_des", Float32)
        self.forearmDesPub = rospy.Publisher("forearm_des", Float32)
        self.wristDesPub = rospy.Publisher("wrist_des", Float32)
        self.palmDesPub = rospy.Publisher("palm_des", Float32)
        self.gripperDesPub = rospy.Publisher("gripper_des", Float32)

        self.armPosSub = rospy.Subscriber("arm_pos", Int16, armPosCb)


    def millis(self):

        return int(1000 * (self.rospy.get_time() - self.init_time))


    def armPosCb(self, data):

        self.arm_pos = data.data
        self.arm_des = self.values[self.arm_pos][0]
        self.forearm_des = self.values[self.arm_pos][1]
        self.wrist_des = self.values[self.arm_pos][2]
        self.palm_des = self.values[self.arm_pos][3]
        self.gripper_des = self.values[self.arm_pos][4]


    def update(self):

        self.armDesPub.publish(self.arm_des)
        self.forearmDesPub.publish(self.forearm_des)
        self.wristDesPub.publish(self.wrist_des)
        self.palmDesPub.publish(self.palm_des)
        self.gripperDesPub.publish(self.gripper_des)


    def spin(self):

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':

    """ main """
    arm_fixed_node = Arm_fixed_node()
    arm_fixed_node.spin()
    
