#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

from ino_mod import *

class Control_interface:
    
    def __init__(self, node_name_override = 'control_interface'):
		
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("control_interface starting with name %s", self.nodename) 

        self.rate = rospy.get_param("param_global_rate", 10)

        self.axes_names     = {'left_stick_hor':0, 'left_stick_ver':1, 'LT':2, 'right_stick_hor':3, 
                            'right_stick_ver':4, 'RT':5, 'cross_hor':6, 'cross_ver':7}
        self.buttons_names  = {'A':0, 'B':1, 'X':2, 'Y':3, 'LB':4, 'RB':5, 'back':6, 'start':7, 'power':8, 
                            'btn_stick_left':9, 'btn_stick_right':10}

        self.veldespub  = rospy.Publisher("vel_delante_des", Float32)
        self.angdespub  = rospy.Publisher("ang_lateral_des", Float32)

        self.angle_des  = 0
        self.vel_des    = 0

        self.joySub = rospy.Subscriber("joy", Joy, self.joyCb)        

    def joyCb(self, data):
		
        self.angle_des  = constrain(data.axes[self.axes_names['left_stick_hor']] / 2, -0.4, 0.4)
        self.vel_des   = data.axes[self.axes_names['right_stick_ver']] * 5
        
    def spin(self):
		
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.angdespub.publish(self.angle_des)
            self.veldespub.publish(self.vel_des);
            r.sleep()

if __name__ == '__main__':
    """ main """
    control_interface = Control_interface()
    control_interface.spin()
