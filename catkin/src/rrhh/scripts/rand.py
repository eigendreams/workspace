#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
import random
from geometry_msgs.msg import Twist

class Publish_velocity:
    
    def __init__(self, node_name_override = 'publish_velocity'):
		
		# The user should be able to change the default name for the node
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        # Simple log
        rospy.loginfo("publish_velocity starting with name %s", self.nodename) 
        # The user should be able to override the output frequency
        self.rate = rospy.get_param("publish_velocity_rate", 2)
        
        # Setting up the output data type
        self.publish_velocity_val = Twist()
        self.publish_velocity_pub = rospy.Publisher('turtle1/cmd_vel', Twist)
        
    def update(self):
		
		self.publish_velocity_val.linear.x = random.random()
		self.publish_velocity_val.angular.z = -1 + 2 * random.random()
		self.publish_velocity_pub.publish(self.publish_velocity_val)
        
                
    def spin(self):

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

if __name__ == '__main__':
    """ main """
    publish_velocity = Publish_velocity()
    publish_velocity.spin()
    
