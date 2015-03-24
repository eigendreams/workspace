#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy

from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

from ino_mod import *

class Control_interface:
    #
    def __init__(self, node_name_override = 'control_interface'):
		#
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("Node starting with name %s", self.nodename) 
        self.rate = float(rospy.get_param("rate", 5))
        #
        # I'm using the names as defined in the ros wiki, seems fine to me
        self.axes_names     = {'left_stick_hor':0, 'left_stick_ver':1, 'LT':2, 'right_stick_hor':3, 'right_stick_ver':4, 'RT':5, 'cross_hor':6, 'cross_ver':7}
        self.buttons_names  = {'A':0, 'B':1, 'X':2, 'Y':3, 'LB':4, 'RB':5, 'back':6, 'start':7, 'power':8, 'btn_stick_left':9, 'btn_stick_right':10}
        #
        self.m1    = rospy.Publisher( "m1",    Int16)              # salida al motor 1
        self.m2    = rospy.Publisher( "m2",    Int16)              # salida al motor 1
        #
        self.angle_des  = 0
        self.angle_des_change = 0
        self.vel_des    = 0
        #
        self.inittime = rospy.get_time()
        self.timelastjoy = -1000
        #
        self.timed_out = True
        #
        self.joySub = rospy.Subscriber("joy", Joy, self.joyCb)        
        #
    def joyCb(self, data):
		#
        self.timelastjoy = millis(self.inittime)
        #
        self.timed_out   = False
        #
        # from -1 to +1, gives from -0.4 to 0.4
        self.angle_des_change   = -map(data.axes[self.axes_names['left_stick_hor']], -1, 1, -5, 5)
        # from -1 to +1, gives -4 to 4
        self.vel_des     = data.axes[self.axes_names['right_stick_ver']] * 13

        #
    def update(self):
        #
        # we timed out! 2s to give room to wifi and processing delays, skip publishing, maybe control was disconnected?
        """
        if ((millis(self.inittime) - self.timelastjoy) > 2000):
            if not self.timed_out :
                self.veldespub.publish(0)
                self.angdespub.publish(0)
            self.timed_out = True
            return
        """
        #
        self.m1.publish(constrain(self.vel_des * 100 - self.angle_des_change * 100,-2000, 2000))
        self.m2.publish(constrain(self.vel_des * 100 + self.angle_des_change * 100,-2000, 2000))
        #
    def spin(self):
		#
        r = rospy.Rate(self.rate)
        #
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
        #
if __name__ == '__main__':
    """ main """
    control_interface = Control_interface()
    control_interface.spin()