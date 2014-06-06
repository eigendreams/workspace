#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from rrhh.msg import RRHH

# We use a multiarray to send data to the robot controller
from numpy import array
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import MultiArrayLayout

# This codes an array containing a linear and angular velocities, and transforms that to
# appropiate PWM outputs for each motor of the RRHH
class RRHH_tf:
    
    def __init__(self, node_name_override = 'rrhh_tf'):
		
		# The user should be able to change the default name for the node
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        # Simple log
        rospy.loginfo("rrhh_tf starting with name %s", self.nodename) 
        # The user should be able to override the output frequency
        self.rate = rospy.get_param("rrhh_tf_rate", 10)
        self.joy_vdes_Sub = rospy.Subscriber("joy_vdes", Float32MultiArray, self.joyvdesCb)
        
        # Setting up the output data type
        self.wheelData = RRHH()
        self.wheelData.wheel1 = 0
        self.wheelData.wheel2 = 0
        self.wheelData.wheel3 = 0
        self.wheelData.wheel4 = 0
        self.wheelPub  = rospy.Publisher("tf_vdes", RRHH)          
        
    def joyvdesCb(self, data):
		      
        # Wheel speeds for the differential case are calculated, data.data contains:
        # [Float32_linear, Float32_angular]
        self.m_right = (data.data[0] + data.data[1] / 2.) * 85
        self.m_left = (data.data[0] - data.data[1] / 2.) * 85
        self.diff = self.m_right - self.m_left
        
        if (self.diff > 0) :
			self.v_mot1 = int(self.m_right)
			self.v_mot2 = int(self.m_right * (85 - self.diff) / 85.)
			self.v_mot3 = int(self.m_right)
			self.v_mot4 = int(self.m_left)
        else :
            self.v_mot1 = int(self.m_left * (85 + self.diff) / 85.)
            self.v_mot2 = int(self.m_left)
            self.v_mot3 = int(self.m_right)
            self.v_mot4 = int(self.m_left)
        
        # Logging and publishing
        rospy.loginfo("RRHH m1: %s m2: %s m3: %s m4: %s", self.v_mot1, self.v_mot2, self.v_mot3, self.v_mot4) 
        
        self.wheelData.wheel1 = self.v_mot1
        self.wheelData.wheel2 = self.v_mot2
        self.wheelData.wheel3 = self.v_mot3
        self.wheelData.wheel4 = self.v_mot4
        
        self.wheelPub.publish(self.wheelData)

if __name__ == '__main__':
    """ main """
    rrhh_tf = RRHH_tf()
    rospy.spin()
    


