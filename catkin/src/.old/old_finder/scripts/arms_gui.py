#!/usr/bin/env python
# -*- coding: utf8 -*-

from gi.repository import Gtk
import gobject

import rospy
import roslib

from finder.msg import FourArmsInt16

class Finder_arms_gui:
    
    def __init__(self, node_name_override = 'finder_arms_gui'):
        
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("finder_arms_gui starting with name %s", self.nodename)
        
        self.armsData = FourArmsInt16()
        self.armsData.arm1 = 0
        self.armsData.arm2 = 0
        self.armsData.arm3 = 0
        self.armsData.arm4 = 0
        self.armsPub  = rospy.Publisher("arms", FourArmsInt16)

        str = roslib.packages.get_pkg_dir('finder') + "/scripts/arms_gui.glade"
        self.gladefile = str

        builder = Gtk.Builder()
        builder.add_from_file(self.gladefile)
        builder.connect_signals(self)

        self.gui_d1 = builder.get_object("arm1")
        self.gui_d1.set_range(-511, 511)
        self.gui_d1.set_value(0)
        self.gui_d1.set_digits(0)
        
        self.gui_d2 = builder.get_object("arm2")
        self.gui_d2.set_range(-511, 511)
        self.gui_d2.set_value(0)
        self.gui_d2.set_digits(0)
        
        self.gui_d3 = builder.get_object("arm3")
        self.gui_d3.set_range(-511, 511)
        self.gui_d3.set_value(0)
        self.gui_d3.set_digits(0)
        
        self.gui_d4 = builder.get_object("arm4")
        self.gui_d4.set_range(-511, 511)
        self.gui_d4.set_value(0)
        self.gui_d4.set_digits(0)

        self.window = builder.get_object("window1")
        self.window.show_all()
        
    def constrain(self, value):
		if (value > 511):
			return 511
		if (value < -511):
			return -511
		return value

    def main(self):
        Gtk.main()

    def on_window1_delete_event(self, widget, data = None):
        Gtk.main_quit()

    def arm1_change_value_cb(self, scroll, value, widget):
        self.armsData.arm1 = self.constrain(int(widget))
    
    def arm2_change_value_cb(self, scroll, value, widget):
        self.armsData.arm2 = self.constrain(int(widget))
        
    def arm3_change_value_cb(self, scroll, value, widget):
        self.armsData.arm3 = self.constrain(int(widget))
        
    def arm4_change_value_cb(self, scroll, value, widget):
        self.arsmData.arm4 = self.constrain(int(widget))
    
    def arm_timeout(self):
        if not rospy.is_shutdown():
            self.armsPub.publish(self.armsData)
        return 1
        
if __name__ == "__main__":
    finder_arms_gui = Finder_arms_gui()
    gobject.timeout_add(50, finder_arms_gui.arm_timeout)
    finder_arms_gui.main()
