#!/usr/bin/env python
# -*- coding: utf8 -*-

from gi.repository import Gtk
import gobject

import rospy
import roslib

from std_msgs.msg import Int16
from sensor_msgs.msg import Joy

from os.path import expanduser

class Finderv2TestingInterface:
    
    def __init__(self, node_name_override = 'finderv2_testing_interface'):
        
        rospy.init_node(node_name_override)
        #self.nodename = rospy.get_name()
        #rospy.loginfo("finder starting with name %s", self.nodename)

        self.mode = False

        self.joy_Sub = rospy.Subscriber("joy", Joy, self.joyCb)
        
        self.pub1 = rospy.Publisher("motor_arm_base", Int16)
        self.pub2 = rospy.Publisher("motor_arm_1", Int16)
        self.pub3 = rospy.Publisher("motor_arm_2", Int16)
        self.pub4 = rospy.Publisher("servo_arm_1", Int16)
        self.pub5 = rospy.Publisher("servo_arm_2", Int16)
        self.pub6 = rospy.Publisher("servo_arm_gripper", Int16)
        self.pub7 = rospy.Publisher("motor_traction_left", Int16)
        self.pub8 = rospy.Publisher("motor_traction_right", Int16)
        self.pub9 = rospy.Publisher("motor_traction_arm_fr", Int16)
        self.pub10 = rospy.Publisher("motor_traction_arm_fl", Int16)
        self.pub11 = rospy.Publisher("motor_traction_arm_br", Int16)
        self.pub12 = rospy.Publisher("motor_traction_arm_bl", Int16)

        self.motor_arm_base_value = 0
        self.motor_arm_1_value = 0
        self.motor_arm_2_value = 0
        self.servo_arm_1_value = 0
        self.servo_arm_2_value = 0
        self.servo_arm_gripper_value = 0
        self.motor_traction_left_value = 0
        self.motor_traction_right_value = 0
        self.motor_traction_arm_fr_value = 0
        self.motor_traction_arm_fl_value = 0
        self.motor_traction_arm_br_value = 0
        self.motor_traction_arm_bl_value = 0

        self.left_trigger = 0
    	self.right_trigger = 0
    	self.trigger = 0
    	self.left_button = 0
    	self.right_button = 0
    	self.a_button = 0
    	self.vertical_arrow = 0
    	self.horizontal_arrow = 0
        self.left_pad = 0
        self.right_pad = 0

        #str = roslib.packages.get_pkg_dir('finder') + "/scripts/finderv2_testing_interface.glade"
        #fix for user path, fix better
        str = expanduser("~") + "/workspace/catkin/src/finderv2/scripts/finderv2_testing_interface.glade"
        self.gladefile = str

        builder = Gtk.Builder()
        builder.add_from_file(self.gladefile)
        builder.connect_signals(self)

        # self.gui_d1 = builder.get_object("d1")
        # self.motor_arm_base_leftpad = builder.get_object("motor_arm_base_leftpad")
        # self.motor_arm_1_leftpad = builder.get_object("motor_arm_1_leftpad")
        # self.motor_arm_2_leftpad = builder.get_object("motor_arm_2_leftpad")
        # self.servo_arm_1_leftpad = builder.get_object("servo_arm_1_leftpad")
        # self.servo_arm_2_leftpad = builder.get_object("servo_arm_2_leftpad")
        # self.servo_arm_gripper_leftpad = builder.get_object("servo_arm_gripper_leftpad")

        # self.motor_traction_arm_fr_leftpad = builder.get_object("motor_traction_arm_fr_leftpad")
        # self.motor_traction_arm_fl_leftpad = builder.get_object("motor_traction_arm_fl_leftpad")
        # self.motor_traction_arm_br_leftpad = builder.get_object("motor_traction_arm_br_leftpad")
        # self.motor_traction_arm_bl_leftpad = builder.get_object("motor_traction_arm_bl_leftpad")

        # self.motor_traction_left_leftpad = builder.get_object("motor_traction_left_leftpad")
        # self.motor_traction_right_leftpad = builder.get_object("motor_traction_right_leftpad")


        # self.motor_arm_base_rightpad = builder.get_object("motor_arm_base_rightpad")
        # self.motor_arm_1_rightpad = builder.get_object("motor_arm_1_rightpad")
        # self.motor_arm_2_rightpad = builder.get_object("motor_arm_2_rightpad")
        # self.servo_arm_1_rightpad = builder.get_object("servo_arm_1_rightpad")
        # self.servo_arm_2_rightpad = builder.get_object("servo_arm_2_rightpad")
        # self.servo_arm_gripper_rightpad = builder.get_object("servo_arm_gripper_rightpad")

        # self.motor_traction_arm_fr_rightpad = builder.get_object("motor_traction_arm_fr_rightpad")
        # self.motor_traction_arm_fl_rightpad = builder.get_object("motor_traction_arm_fl_rightpad")
        # self.motor_traction_arm_br_rightpad = builder.get_object("motor_traction_arm_br_rightpad")
        # self.motor_traction_arm_bl_rightpad = builder.get_object("motor_traction_arm_bl_rightpad")

        # self.motor_traction_left_rightpad = builder.get_object("motor_traction_left_rightpad")
        # self.motor_traction_right_rightpad = builder.get_object("motor_traction_right_rightpad")

        self.window = builder.get_object("window1")
        self.window.show_all()

    def main(self):
    	Gtk.main()

    def joyCb(self, data):
    	self.left_trigger = (data.axes[2]-1)*-50
    	self.right_trigger = (data.axes[5]-1)*50
    	self.trigger = self.left_trigger + self.right_trigger
    	self.left_button = data.buttons[4] 
    	self.right_button = data.buttons[5]
    	self.a_button = data.buttons[0]
    	self.vertical_arrow = data.axes[7]
    	self.horizontal_arrow = data.axes[6]
        self.left_pad = data.axes[1] * 100
        self.right_pad = data.axes[4] * 100

        if self.a_button == 1:
        	self.mode = not self.mode

		# self.motor_arm_base_value = 0
		# self.motor_arm_1_value = 0
		# self.motor_arm_2_value = 0
		# self.servo_arm_1_value = 0
		# self.servo_arm_2_value = 0
		# self.servo_arm_gripper_value = 0
		# self.motor_traction_left_value = 0
		# self.motor_traction_right_value = 0
		# self.motor_traction_arm_fr_value = 0
		# self.motor_traction_arm_fl_value = 0
		# self.motor_traction_arm_br_value = 0
		# self.motor_traction_arm_bl_value = 0
        
    def on_window1_delete_event(self, widget, data = None):
		Gtk.main_quit()
    
    def interface_timeout(self):

		# if not rospy.is_shutdown():
		# 	self.motor_arm_base_value = 0
		# 	self.motor_arm_1_value = 0
		# 	self.motor_arm_2_value = 0
		# 	self.servo_arm_1_value = 0
		# 	self.servo_arm_2_value = 0
		# 	self.servo_arm_gripper_value = 0
		# 	self.motor_traction_left_value = 0
		# 	self.motor_traction_right_value = 0
		# 	self.motor_traction_arm_fr_value = 0
		# 	self.motor_traction_arm_fl_value = 0
		# 	self.motor_traction_arm_br_value = 0
		# 	self.motor_traction_arm_bl_value = 0

		if self.mode == 0:
			if self.left_button == True:
				self.motor_traction_arm_fr_value = self.right_pad
				self.motor_traction_arm_fl_value = self.left_pad
			if self.right_button == True:
				self.motor_traction_arm_br_value = self.right_pad
				self.motor_traction_arm_bl_value = self.left_pad
			if self.left_button == False and self.right_button == False:
				self.motor_traction_left_value = self.right_pad
				self.motor_traction_right_value = self.left_pad
		else:
			self.motor_arm_base_value = self.trigger
			self.motor_arm_1_value = self.left_pad*-1
			self.motor_arm_2_value = self.right_pad*-1
			self.servo_arm_1_value = self.left_button - self.right_button
			self.servo_arm_2_value = self.vertical_arrow
			self.servo_arm_gripper_value = self.horizontal_arrow

		self.pub1.publish(self.motor_arm_base_value)
		self.pub2.publish(self.motor_arm_1_value)
		self.pub3.publish(self.motor_arm_2_value)
		self.pub4.publish(self.servo_arm_1_value)
		self.pub5.publish(self.servo_arm_2_value)
		self.pub6.publish(self.servo_arm_gripper_value)
		self.pub7.publish(self.motor_traction_left_value)
		self.pub8.publish(self.motor_traction_right_value)
		self.pub9.publish(self.motor_traction_arm_fr_value)
		self.pub10.publish(self.motor_traction_arm_fl_value)
		self.pub11.publish(self.motor_traction_arm_br_value)
		self.pub12.publish(self.motor_traction_arm_bl_value)

		print "Inicio"
		print self.motor_arm_base_value
		print self.motor_arm_1_value
		print self.motor_arm_2_value
		print self.servo_arm_1_value
		print self.servo_arm_2_value
		print self.servo_arm_gripper_value
		print self.motor_traction_left_value
		print self.motor_traction_right_value
		print self.motor_traction_arm_fr_value
		print self.motor_traction_arm_fl_value
		print self.motor_traction_arm_br_value
		print self.motor_traction_arm_bl_value
		print self.mode

		print "Ejes"
		print self.left_trigger
		print self.right_trigger 
		print self.trigger
		print self.left_button
		print self.right_button
		print self.a_button
		print self.vertical_arrow
		print self.horizontal_arrow
		print self.left_pad
		print self.right_pad
			
		return 1
        
if __name__ == "__main__":
    finderv2_testing_interface = Finderv2TestingInterface()    
    gobject.timeout_add(100, finderv2_testing_interface.interface_timeout)
    finderv2_testing_interface.main()

