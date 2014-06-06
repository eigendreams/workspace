#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy

from math import exp
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

# We use a multiarray to send data to the robot controller script
from numpy import array
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import MultiArrayLayout

# This code listen to an input joystick and generates linear and angular velocities based on the state
# of the buttons, sticks and triggers. Extra "just in case" nodes are used in the microcontroller to 
# ensure proper behaviour
class RRHH_joy:
    
    def __init__(self, node_name_override = 'rrhh_joy'):
		
		# The user should be able to change the default name for the node
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        # Simple log
        rospy.loginfo("rrhh_joy starting with name %s", self.nodename) 
        # The user should be able to override the output frequency
        self.rate = rospy.get_param("rrhh_joy_rate", 10)
        # The buttons and axis used in the joystick can be overriden too
        self.axes_names = {'linear':5, 'angular':0}
        self.buttons_names = {'reverse':1, 'start':8, 'stop':2}
        # Subscription to the joystick node, calling the joyCb function
        self.joy_Sub = rospy.Subscriber("joy", Joy, self.joyCb)
        
        # Setting up the output data type
        self.vals_joy_vdes = MultiArrayDimension()
        self.layout_joy_vdes = MultiArrayLayout()
        self.data_joy_vdes = []
        self.vals_joy_vdes.label = "joy_vdes"
        self.vals_joy_vdes.size = 2
        self.vals_joy_vdes.stride = 2
        self.layout_joy_vdes.data_offset = 2
        self.layout_joy_vdes.dim.append(self.vals_joy_vdes)
        self.joy_vdes_Pub = rospy.Publisher('joy_vdes', Float32MultiArray)
        
        # Control topics, used to change controller behaviour, Mc indicates if the values are to be
        # passed as desired speeds or understood as PWM outputs. Mr switches a ON/OFF relay, Mc is NOT
        # implemented yet in the microcontroller
        self.Mc_Pub = rospy.Publisher("mc", Bool)
        self.Mr_Pub = rospy.Publisher("mr", Bool)
        
        # Internal variables
        self.reverse = False
        self.mult_reverse = 0
        self.angular_rate = 0
        self.linear_rate = 0
        self.exp_term_angular_rate = 0
        self.exp_term_lin_prod = 0   
        self.last_lin_prod = 0
        self.last_angular_rate = 0               
        self.start_flag = False
        
    def joyCb(self, data):
		
        self.reverse = data.buttons[self.buttons_names['reverse']]
        self.angular_rate = data.axes[self.axes_names['angular']]
        self.linear_rate = (1 - data.axes[self.axes_names['linear']]) / 2.
        
        if (data.buttons[self.buttons_names['start']]):
            self.Mr_Pub.publish(True)
            self.start_flag = True
        if (data.buttons[self.buttons_names['stop']]):
            self.Mr_Pub.publish(False)
            self.start_flag = False
            
        if not (self.start_flag):
			self.angular_rate = 0
			self.linear_rate = 0
        
    def update(self):
		
        if (self.reverse):
            self.mult_reverse = -1.
        else:
            self.mult_reverse = 1.
        
        self.lin_prod = self.mult_reverse * self.linear_rate
        
        # The scripts "forgets" past data exponentially, to smooth abrupt changes
        self.now_dif_lin_prod = self.last_lin_prod - self.lin_prod
        self.exp_term_lin_prod = (self.exp_term_lin_prod + self.now_dif_lin_prod) * exp(-3. * (1. / self.rate))
        self.out_lin_prod = self.lin_prod + self.exp_term_lin_prod
        self.last_lin_prod = self.lin_prod
        
        self.now_dif_angular_rate = self.last_angular_rate - self.angular_rate
        self.exp_term_angular_rate = (self.exp_term_angular_rate + self.now_dif_angular_rate) * exp(-3. * (1. / self.rate))
        self.out_angular_rate = self.angular_rate + self.exp_term_angular_rate
        self.last_angular_rate = self.angular_rate
        
        # Logging and publishing
        rospy.loginfo("RRHH linear: %2.3f angular: %2.3f", self.out_lin_prod, self.out_angular_rate) 
        self.data_joy_vdes = [self.out_lin_prod, self.out_angular_rate]
        self.joy_vdes_Pub.publish(self.layout_joy_vdes, self.data_joy_vdes)
        
    def initNodes(self):
		
        # Basic init setup, the values obtained are to be used as PWM outputs and the robot should be OFF.
        self.Mc_Pub.publish(True)
        self.Mr_Pub.publish(False)

    def spin(self):
		
        self.initNodes()
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

if __name__ == '__main__':
    """ main """
    rrhh_joy = RRHH_joy()
    rrhh_joy.spin()
    

