#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from finder.msg import TwoWheelInt16

class Interface_diff:
    
    def __init__(self, node_name_override = 'interface_diff'):
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("interface_diff starting with name %s", self.nodename)
        self.rate = rospy.get_param("interface_diff_rate", 10)
        
        self.wheelData = TwoWheelInt16()
        self.wheelData.wheel1 = 0
        self.wheelData.wheel2 = 0
        self.wheelPub  = rospy.Publisher("traction_motors", TwoWheelInt16)
        
        self.mtr = rospy.Subscriber("motor_traction_right", Int16, self.cbmtr)
        self.mtl = rospy.Subscriber("motor_traction_left", Int16, self.cbmtl)
    
    def cbmtr(self, data):
        self.wheelData.wheel2  = data.data/1
    def cbmtl(self, data):
        self.wheelData.wheel1 = -data.data/1
        
    def update(self):
        self.wheelPub.publish(self.wheelData)

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

if __name__ == '__main__':
    """ main """
    interface_diff = Interface_diff()
    interface_diff.spin()
    
