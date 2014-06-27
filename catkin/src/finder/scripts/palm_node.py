#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from math import pi


class Palm_node:

    def __init__(self, node_name_override = 'palm_node'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("palm_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)
        self.init_time = rospy.get_time()
        self.palm_des = 0
        self.palmOutPub = rospy.Publisher("palm_out", Int16)
        self.palmDesSub = rospy.Subscriber("palm_des", Float32, self.palmDesCb)


    def palmDesCb(self, data):

        self.palm_des = data.data


    def update(self):

        self.palmOutPub.publish(self.palm_des)


    def spin(self):

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':

    """ main """
    palm_node = Palm_node()
    palm_node.spin()
    
