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
        # I think, what happens if comm is lost? then this comm channels should stop
        self.veldespub  = rospy.Publisher("vel_delante_des", Float32)
        self.angdespub  = rospy.Publisher("ang_lateral_des", Float32)
        #
        self.angmult    = rospy.Publisher("ang_mult", Float32)
        self.velmult    = rospy.Publisher("vel_mult", Float32)
        #
        self.alpub      = rospy.Publisher("al", Int16)
        #
        self.conmode    = rospy.Publisher("con_mode", Int16)
        #
        self.btog = 0
        self.lastb = 0
        self.lastbtog = 0
        self.btogchanged = 0
        #
        self.rbtog = 0
        self.lbtog = 0
        self.lastrb = 0
        self.lastlb = 0
        #
        self.rtval = 0
        self.ltval = 0
        #
        self.powtog = 0
        self.lastpow = 0
        self.lastpowtog = 0
        self.powtogchanged = 0
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
        self.angdifval = 0
        #
        self.joySub = rospy.Subscriber("joy", Joy, self.joyCb) 
        #
        self.angdifsub = rospy.Subscriber( "angdif", Float32, self.angdifcb)       
        #
    def angdifcb(self, data):
        #
        self.angdifval = data.data
        #
    def joyCb(self, data):
		#
        """
        self.timelastjoy = millis(self.inittime)
        #
        self.timed_out   = False
        """
        #
        self.lastpowtog = self.powtog
        if (data.buttons[self.buttons_names['power']] is 1 and self.lastpow is 0):
            self.powtog = int(not self.powtog)
        self.powtogchanged = self.lastpowtog or not (self.powtog == self.lastpowtog)
        self.lastpow = data.buttons[self.buttons_names['power']]
        #
        #
        self.lastbtog = self.btog
        if (data.buttons[self.buttons_names['B']] is 1 and self.lastb is 0):
            self.btog = int(not self.btog)
        self.btogchanged = 0 or not (self.btog == self.lastbtog)
        self.lastb = data.buttons[self.buttons_names['B']]
        #
        self.rtval = (data.axes[self.axes_names['RT']] - 1) / -2.
        self.ltval = (data.axes[self.axes_names['LT']] - 1) / -2.
        #
        self.angle_des_change   = -data.axes[self.axes_names['left_stick_hor']]
        #
        self.vel_des     = data.axes[self.axes_names['right_stick_ver']]
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
        # publish the control mode, if 0 control is used, if 1 pwm values are passed directly and the motor control node waits 
        self.conmode.publish(self.btog)
        #
        self.angmultval = constrain( (1 + self.ltval) * (1 + self.lastlb), 1, 4)
        self.velmultval = constrain( (1 + self.rtval) * (1 + self.lastrb), 1, 4)
        # the multiply values have to published anyway, yo be cathced and processed in the control nodes
        self.angmult.publish(self.angmultval)
        self.velmult.publish(self.velmultval)
        #
        # we wish to send pwm directly or send control commands
        #
        # if toggled, reset multipliers buttons to 0
        #if (self.btogchanged):
        #    self.lbtog = 0
        #    self.rbtog = 0
        #    self.lastlb = 0
        #    self.lastrb = 0
            #self.btogchanged = 0
        #
        #
        #if (self.powtogchanged):
        #    self.m1.publish(0);
        #    self.m2.publish(0);
        #    self.alpub.publish(1)
        #    self.powtogchanged = 0
        #    return
        #else:
        self.alpub.publish(self.powtog)
        #
        #
        if (self.btog is 1):
            #
            if (self.angdifval > 0.41):
                self.angle_des_change = constrain(self.angle_des_change, -30, 0)
            if (self.angdifval < -0.41):
                self.angle_des_change = constrain(self.angle_des_change, 0, 30)
            #
            self.m1.publish(constrain(self.vel_des * 6 * self.velmultval * 100 - self.angle_des_change * 6 * self.angmultval * 100,-3000, 3000))
            self.m2.publish(constrain(self.vel_des * 6 * self.velmultval * 100 + self.angle_des_change * 6 * self.angmultval * 100,-3000, 3000))
        else:
            self.angdespub.publish(self.angle_des_change * 0.4)
            self.veldespub.publish(self.vel_des   * 1.0)
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
