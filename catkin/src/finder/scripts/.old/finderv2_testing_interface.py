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

        self.leftpad = 0
        self.rightpad = 0

        #str = roslib.packages.get_pkg_dir('finder') + "/scripts/finderv2_testing_interface.glade"
        #fix for user path, fix better
        str = expanduser("~") + "/workspace/catkin/src/finder/scripts/finderv2_testing_interface.glade"
        self.gladefile = str

        builder = Gtk.Builder()
        builder.add_from_file(self.gladefile)
        builder.connect_signals(self)

        # self.gui_d1 = builder.get_object("d1")
        self.motor_arm_base_leftpad = builder.get_object("motor_arm_base_leftpad")
        self.motor_arm_1_leftpad = builder.get_object("motor_arm_1_leftpad")
        self.motor_arm_2_leftpad = builder.get_object("motor_arm_2_leftpad")
        self.servo_arm_1_leftpad = builder.get_object("servo_arm_1_leftpad")
        self.servo_arm_2_leftpad = builder.get_object("servo_arm_2_leftpad")
        self.servo_arm_gripper_leftpad = builder.get_object("servo_arm_gripper_leftpad")

        self.motor_traction_arm_fr_leftpad = builder.get_object("motor_traction_arm_fr_leftpad")
        self.motor_traction_arm_fl_leftpad = builder.get_object("motor_traction_arm_fl_leftpad")
        self.motor_traction_arm_br_leftpad = builder.get_object("motor_traction_arm_br_leftpad")
        self.motor_traction_arm_bl_leftpad = builder.get_object("motor_traction_arm_bl_leftpad")

        self.motor_traction_left_leftpad = builder.get_object("motor_traction_left_leftpad")
        self.motor_traction_right_leftpad = builder.get_object("motor_traction_right_leftpad")


        self.motor_arm_base_rightpad = builder.get_object("motor_arm_base_rightpad")
        self.motor_arm_1_rightpad = builder.get_object("motor_arm_1_rightpad")
        self.motor_arm_2_rightpad = builder.get_object("motor_arm_2_rightpad")
        self.servo_arm_1_rightpad = builder.get_object("servo_arm_1_rightpad")
        self.servo_arm_2_rightpad = builder.get_object("servo_arm_2_rightpad")
        self.servo_arm_gripper_rightpad = builder.get_object("servo_arm_gripper_rightpad")

        self.motor_traction_arm_fr_rightpad = builder.get_object("motor_traction_arm_fr_rightpad")
        self.motor_traction_arm_fl_rightpad = builder.get_object("motor_traction_arm_fl_rightpad")
        self.motor_traction_arm_br_rightpad = builder.get_object("motor_traction_arm_br_rightpad")
        self.motor_traction_arm_bl_rightpad = builder.get_object("motor_traction_arm_bl_rightpad")

        self.motor_traction_left_rightpad = builder.get_object("motor_traction_left_rightpad")
        self.motor_traction_right_rightpad = builder.get_object("motor_traction_right_rightpad")

        self.window = builder.get_object("window1")
        self.window.show_all()

    def main(self):
        Gtk.main()

    def joyCb(self, data):
        self.leftpad = data.axes[1] * 100
        self.rightpad = data.axes[4] * 100
        
    def on_window1_delete_event(self, widget, data = None):
		Gtk.main_quit()
    
    def interface_timeout(self):
        if not rospy.is_shutdown():
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

			# LeftPad verifications
			if(self.motor_arm_base_leftpad.get_active()):
				self.motor_arm_base_value = self.leftpad
			if(self.motor_arm_1_leftpad.get_active()):
				self.motor_arm_1_value = self.leftpad
			if(self.motor_arm_2_leftpad.get_active()):
				self.motor_arm_2_value = self.leftpad
			if(self.servo_arm_1_leftpad.get_active()):
				self.servo_arm_1_value = self.leftpad
			if(self.servo_arm_2_leftpad.get_active()):
				self.servo_arm_2_value = self.leftpad
			if(self.servo_arm_gripper_leftpad.get_active()):
				self.servo_arm_gripper_value = self.leftpad
			if(self.motor_traction_left_leftpad.get_active()):
				self.motor_traction_left_value = self.leftpad
			if(self.motor_traction_right_leftpad.get_active()):
				self.motor_traction_right_value = self.leftpad
			if(self.motor_traction_arm_fr_leftpad.get_active()):
				self.motor_traction_arm_fr_value = self.leftpad
			if(self.motor_traction_arm_fl_leftpad.get_active()):
				self.motor_traction_arm_fl_value = self.leftpad
			if(self.motor_traction_arm_br_leftpad.get_active()):
				self.motor_traction_arm_br_value = self.leftpad
			if(self.motor_traction_arm_bl_leftpad.get_active()):
				self.motor_traction_arm_bl_value = self.leftpad
			
			# Rightpad verifications
			if(self.motor_arm_base_rightpad.get_active()):
				self.motor_arm_base_value = self.rightpad
			if(self.motor_arm_1_rightpad.get_active()):
				self.motor_arm_1_value = self.rightpad
			if(self.motor_arm_2_rightpad.get_active()):
				self.motor_arm_2_value = self.rightpad
			if(self.servo_arm_1_rightpad.get_active()):
				self.servo_arm_1_value = self.rightpad
			if(self.servo_arm_2_rightpad.get_active()):
				self.servo_arm_2_value = self.rightpad
			if(self.servo_arm_gripper_rightpad.get_active()):
				self.servo_arm_gripper_value = self.rightpad
			if(self.motor_traction_left_rightpad.get_active()):
				self.motor_traction_left_value = self.rightpad
			if(self.motor_traction_right_rightpad.get_active()):
				self.motor_traction_right_value = self.rightpad
			if(self.motor_traction_arm_fr_rightpad.get_active()):
				self.motor_traction_arm_fr_value = self.rightpad
			if(self.motor_traction_arm_fl_rightpad.get_active()):
				self.motor_traction_arm_fl_value = self.rightpad
			if(self.motor_traction_arm_br_rightpad.get_active()):
				self.motor_traction_arm_br_value = self.rightpad
			if(self.motor_traction_arm_bl_rightpad.get_active()):
				self.motor_traction_arm_bl_value = self.rightpad

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
				
			return 1
        
if __name__ == "__main__":
    finderv2_testing_interface = Finderv2TestingInterface()    
    gobject.timeout_add(100, finderv2_testing_interface.interface_timeout)
    finderv2_testing_interface.main()

