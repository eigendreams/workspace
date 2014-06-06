#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from finder.msg import FourArmsInt16
import sys

class Finder_arms_shell:
    
    def __init__(self, node_name_override = 'finder_arms_shell'):
        
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("finder_arms_shell starting with name %s", self.nodename)
        self.rate = rospy.get_param("finder_arms_shell_rate", 10)
        
        self.armsData = FourArmsInt16()
        self.armsData.arm1 = 0
        self.armsData.arm2 = 0
        self.armsData.arm3 = 0
        self.armsData.arm4 = 0
        self.armsPub  = rospy.Publisher("arms", FourArmsInt16)
            
    def constrain(self, value):
		
        if (value > 511):
            return 511
        if (value < -511):
            return -511
        return value
        
    def update(self):
         
        val = raw_input()
        
        if (val is not None and val is not ""):
			
			vals = val.split()
			
			if (len(vals) >= 1):
				if (vals[0] == "^C"):
					sys.exit(0)
				
			if (len(vals) == 4):
				try:
					self.armsData.arm1 = self.constrain(int(vals[0]))
					self.armsData.arm2 = self.constrain(int(vals[1]))
					self.armsData.arm3 = self.constrain(int(vals[2]))
					self.armsData.arm4 = self.constrain(int(vals[3]))
				except:
					pass
        
        self.armsPub.publish(self.armsData)

    def spin(self):
        
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

if __name__ == '__main__':
    """ main """
    finder_arms_shell = Finder_arms_shell()
    finder_arms_shell.spin()
    
