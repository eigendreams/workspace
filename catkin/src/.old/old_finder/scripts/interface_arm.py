#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from arm_interface.msg import Arm

class Interface_arm:
    
    def __init__(self, node_name_override = 'interface_arm'):
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("interface_arm starting with name %s", self.nodename)
        self.rate = rospy.get_param("interface_arm_rate", 10)
        
        self.armData = Arm()
        self.armData.dof1 = 0
        self.armData.dof2 = 0
        self.armData.dof3 = 0
        self.armData.dof4 = 0
        self.armData.dof5 = 0
        self.armData.dof6 = 0
        self.armPub  = rospy.Publisher("arm_motors", Arm)
        
        self.mab = rospy.Subscriber("motor_arm_base", Int16, self.cbmab)
        self.ma1 = rospy.Subscriber("motor_arm_1", Int16, self.cbma1)
        self.ma2 = rospy.Subscriber("motor_arm_2", Int16, self.cbma2)
        self.sa1 = rospy.Subscriber("servo_arm_1", Int16, self.cbsa1)
        self.sa2 = rospy.Subscriber("servo_arm_2", Int16, self.cbsa2)
        self.sag = rospy.Subscriber("servo_arm_gripper", Int16, self.cbsag)
        
        self.mun1 = 0
        self.mun2 = 0
        self.mun3 = 0
    
    def cbmab(self, data):
        self.armData.dof1 = data.data
    def cbma1(self, data):
        self.armData.dof2 = data.data * .8
    def cbma2(self, data):
        self.armData.dof3 = data.data * .8
    def cbsa1(self, data):
        self.mun1 = self.constrain(self.mun1 + data.data * 10, -127, 127)
        self.armData.dof4 = self.mun1 + 60
    def cbsa2(self, data):
        self.mun2 = self.constrain(self.mun2 + data.data * 10, -90, 90)
        self.armData.dof5 = self.mun2
    def cbsag(self, data):
        self.mun3 = self.constrain(self.mun3 + data.data * 10, -127, 127)
        self.armData.dof6 = self.mun3
        
    def constrain(self, val, min, max):
        if val < min:
            return min
        if val > max:
            return max
        return val
        
    def update(self):
        self.armPub.publish(self.armData)

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

if __name__ == '__main__':
    """ main """
    interface_arm = Interface_arm()
    interface_arm.spin()
    
