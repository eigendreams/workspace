#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16

class Alive:
    #
    def __init__(self, node_name_override = 'alive'):
        #
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("Node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("alive_rate", 1)
        self.alive_val = 1
        self.alivePub = rospy.Publisher('al', Int16)
        #
    def update(self):
        #
        self.alivePub.publish(self.alive_val)
        #
    def spin(self):
		#
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
        #
if __name__ == '__main__':
    """ main """
    alive = Alive()
    alive.spin()
    