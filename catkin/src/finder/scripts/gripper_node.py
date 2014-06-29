#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from math import pi


class Palm_node:

    def __init__(self, node_name_override = 'gripper_node'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("gripper_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)
        self.init_time = rospy.get_time()
        self.gripper_des = 0
        self.gripperOutPub = rospy.Publisher("gripper_out", Int16)
        self.gripperDesSub = rospy.Subscriber("gripper_des", Float32, self.gripperDesCb)


    def gripperDesCb(self, data):

        self.gripper_des = data.data


    def update(self):

        self.gripperOutPub.publish(self.gripper_des * 100)


    def spin(self):

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':

    """ main """
    gripper_node = Palm_node()
    gripper_node.spin()
    
