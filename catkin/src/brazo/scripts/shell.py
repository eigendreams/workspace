#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from arm_interface.msg import Arm
import sys

class arm_interface_shell:
    
    def __init__(self, node_name_override = 'arm_interface_shell'):
        
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("arm_interface_shell starting with name %s", self.nodename)
        self.rate = rospy.get_param("arm_interface_shell_rate", 10)
        
        self.armData = Arm()
        self.armData.dof1 = 0
        self.armData.dof2 = -90
        self.armData.dof3 = -90
        self.armData.dof4 = 0
        self.armData.dof5 = 0
        self.armData.dof6 = 0
        self.armPub  = rospy.Publisher("arm", Arm)
            
    def constrain(self, value):
        if (value > 90):
            return 90
        if (value < -90):
            return -90
        return value
        
    def update(self):
         
        val = raw_input()
        
        if (val is not None and val is not ""):
			
			vals = val.split()
			
			if (len(vals) >= 1):
				if (vals[0] == "^C"):
					sys.exit(0)
				
			if (len(vals) == 6):
				try:
					self.armData.dof1 = self.constrain(int(vals[0]))
					self.armData.dof2 = self.constrain(int(vals[1]))
					self.armData.dof3 = self.constrain(int(vals[2]))
					self.armData.dof4 = self.constrain(int(vals[3]))
					self.armData.dof5 = self.constrain(int(vals[4]))
					self.armData.dof6 = self.constrain(int(vals[5]))
				except:
					pass
			
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
    arm_interface_shell = arm_interface_shell()
    arm_interface_shell.spin()
    
