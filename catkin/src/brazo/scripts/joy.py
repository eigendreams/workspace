#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy

from sensor_msgs.msg import Joy
from arm_interface.msg import Arm

class Arm_interface_joy:
    
    def __init__(self, node_name_override = 'arm_interface_joy'):
		
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("arm_interface_joy starting with name %s", self.nodename)
        
        self.rate = rospy.get_param("arm_interface_joy_rate", 10)
        
        self.axes_names = {'base':5, 'brazo':1, 'ante':4, 'omega':3, 'phi':6}
        self.buttons_names = {'abrir':7, 'cerrar':6, 'reinicio':9}
        self.joy_Sub = rospy.Subscriber("joy", Joy, self.joyCb)
        
        self.armData = Arm()
        self.armData.dof1 = 0
        self.armData.dof2 = -90
        self.armData.dof3 = -90
        self.armData.dof4 = 0
        self.armData.dof5 = 0
        self.armData.dof6 = 0
        self.armPub  = rospy.Publisher("arm", Arm)
        
        self.vd1 = 0
        self.vd2 = 0
        self.vd3 = 0
        self.vd4 = 0
        self.vd5 = 0
        self.vd6 = 0
        
    def joyCb(self, data):
		
		self.vd1 = data.axes[self.axes_names['base']]
		self.vd2 = data.axes[self.axes_names['brazo']]
		self.vd3 = data.axes[self.axes_names['ante']]
		self.vd4 = data.axes[self.axes_names['omega']]
		self.vd5 = data.axes[self.axes_names['phi']]
		
		if (data.buttons[self.button_names['abrir']]):
			self.vd6 = 1
		if (data.buttons[self.button_names['cerrar']]):
			self.vd6 = -1
		
		if (data.buttons[self.button_names['reinicio']]):
			self.armData.dof1 = 0
            self.armData.dof2 = -90
            self.armData.dof3 = -90
            self.armData.dof4 = 0
            self.armData.dof5 = 0
            self.armData.dof6 = 0
			# Just in case, return gripper to neutral position and NULL speed
			self.vd6 = 0
			
	def constrain(self, value):
		if (value > 90):
			return 90
		if (value < -90):
			return -90
		return value
        
    def update(self):
		
		self.armData.dof1 = self.constrain(self.armData.dof1 + self.vd1)
		self.armData.dof2 = self.constrain(self.armData.dof2 + self.vd2)
		self.armData.dof3 = self.constrain(self.armData.dof3 + self.vd3)
		self.armData.dof4 = self.constrain(self.armData.dof4 + self.vd4)
		self.armData.dof5 = self.constrain(self.armData.dof5 + self.vd5)
		self.armData.dof6 = self.constrain(self.armData.dof6 + self.vd6)
		
		# Correct for angle overpass
		if (self.armData.dof2 + self.armData.dof3 > 90):
			self.armData.dof3 = 90 - self.armData.dof2
        
        self.armPub.publish(self.armData)

    def spin(self):
		
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

if __name__ == '__main__':
    """ main """
    arm_interface_joy = Arm_interface_joy()
    arm_interface_joy.spin()
    
