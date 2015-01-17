#!/usr/bin/env python
# -*- coding: utf8 -*-

"""
Some code borrowed from twist_to_motors.py by Jon Stephan
Copyright (C) 2013 Jakob Culebro Reyes
You know the GNU stuff, attribution, no warranty, etc.
"""

import rospy
import roslib
roslib.load_manifest('finder_low_py')

from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import  MultiArrayLayout

from numpy import array
from geometry_msgs.msg import Twist 

class TwistToMotors():

    def __init__(self, node_name_in_twi = "twist_to_motors"):
        rospy.init_node(node_name_in_twi)
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)
    
        self.wheel_radii = rospy.get_param('wheel_radii', 0.254)
        self.base_width = rospy.get_param('base_width', 0.508)
        
        self.vals_vdes = MultiArrayDimension()
        self.layout_vdes = MultiArrayLayout()
        self.data_vdes = []
        self.vals_vdes.label = "vdes"
        self.vals_vdes.size = 2
        self.vals_vdes.stride = 2
        self.layout_vdes.data_offset = 2
        self.layout_vdes.dim.append(self.vals_vdes)
        self.vdesPub = rospy.Publisher('vdes', Float32MultiArray)
        
        rospy.Subscriber('twist', Twist, self.twistCallback)
    
        self.rate = rospy.get_param("twist_to_motors_rate", 20)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        
        self.left = 0
        self.right = 0
        
    def spin(self):
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
    
        ###### main loop  ######
        while not rospy.is_shutdown():
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()
                
    def spinOnce(self):
        # dx = (l + r) / 2
        # dr = (r - l) / w
        self.right = (1.0 * self.dx + self.dr * self.base_width / 2) / self.wheel_radii 
        self.left = (1.0 * self.dx - self.dr * self.base_width / 2) / self.wheel_radii
        # rospy.loginfo("publishing: (%d, %d)", left, right)
        self.data_vdes = [self.right, self.left]
        self.vdesPub.publish(self.layout_vdes, self.data_vdes)
            
        self.ticks_since_target += 1

    def twistCallback(self,msg):
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y
    
if __name__ == '__main__':
    """ main """
    twistToMotors = TwistToMotors()
    twistToMotors.spin()
