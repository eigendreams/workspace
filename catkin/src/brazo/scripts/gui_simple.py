#!/usr/bin/env python
# -*- coding: utf8 -*-

from gi.repository import Gtk
import gobject

import rospy
import roslib

from arm_interface.msg import Arm

class Arm_interface_gui_simple:
    
    def __init__(self, node_name_override = 'arm_interface_gui_simple'):
        
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("arm_interface_gui_simple starting with name %s", self.nodename)
        
        self.armData = Arm()        
        self.armData.dof1 = 0
        self.armData.dof2 = -90
        self.armData.dof3 = -90
        self.armData.dof4 = 0
        self.armData.dof5 = 0
        self.armData.dof6 = 0
        self.armPub  = rospy.Publisher("arm", Arm)

        str = roslib.packages.get_pkg_dir('arm_interface') + "/scripts/gui_simple.glade"
        self.gladefile = str

        builder = Gtk.Builder()
        builder.add_from_file(self.gladefile)
        builder.connect_signals(self)

        self.gui_d1 = builder.get_object("d1")
        self.gui_d1.set_range(-90, 90)
        self.gui_d1.set_value(0)
        self.gui_d1.set_digits(0)
        
        self.gui_d2 = builder.get_object("d2")
        self.gui_d2.set_range(-90, 90)
        self.gui_d2.set_value(-90)
        self.gui_d2.set_digits(0)
        
        self.gui_d3 = builder.get_object("d3")
        self.gui_d3.set_range(-90, 90)
        self.gui_d3.set_value(-90)
        self.gui_d3.set_digits(0)
        
        self.gui_d4 = builder.get_object("d4")
        self.gui_d4.set_range(-90, 90)
        self.gui_d4.set_value(0)
        self.gui_d4.set_digits(0)
        
        self.gui_d5 = builder.get_object("d5")
        self.gui_d5.set_range(-90, 90)
        self.gui_d5.set_value(0)
        self.gui_d5.set_digits(0)
        
        self.gui_d6 = builder.get_object("d6")
        self.gui_d6.set_range(-90, 90)
        self.gui_d6.set_value(0)
        self.gui_d6.set_digits(0)

        self.window = builder.get_object("window1")
        self.window.show_all()
        
    def constrain(self, value):
		if (value > 90):
			return 90
		if (value < -90):
			return -90
		return value

    def main(self):
        Gtk.main()

    def on_window1_delete_event(self, widget, data = None):
        Gtk.main_quit()

    def on_d1_change_value(self, scroll, value, widget):
        self.armData.dof1 = self.constrain(int(widget))
    
    def on_d2_change_value(self, scroll, value, widget):
        self.armData.dof2 = self.constrain(int(widget))
        
    def on_d3_change_value(self, scroll, value, widget):
        self.armData.dof3 = self.constrain(int(widget))
        
    def on_d4_change_value(self, scroll, value, widget):
        self.armData.dof4 = self.constrain(int(widget))
        
    def on_d5_change_value(self, scroll, value, widget):
        self.armData.dof5 = self.constrain(int(widget))
        
    def on_d6_change_value(self, scroll, value, widget):
        self.armData.dof6 = self.constrain(int(widget))
    
    def arm_timeout(self):
        if not rospy.is_shutdown():
            self.armPub.publish(self.armData)
        return 1
        
if __name__ == "__main__":
    arm_interface_gui_simple = Arm_interface_gui_simple()
    gobject.timeout_add(50, arm_interface_gui_simple.arm_timeout)
    arm_interface_gui_simple.main()
