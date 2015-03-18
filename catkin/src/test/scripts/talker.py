#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class Talker:
        def __init__(self, node_name = 'talker'):
                rospy.init_node(node_name)
                self.nodename = rospy.get_name()
                rospy.loginfo("Started node %s", self.nodename)
                self.rate = rospy.Rate(10)
                self.pub = rospy.Publisher("chatter", String)
        def spin(self):
                while not rospy.is_shutdown():
                        self.hello_str = "Hello World %s" % rospy.get_time()
                        rospy.loginfo(self.hello_str)
                        self.pub.publish(self.hello_str)
                        self.rate.sleep()
if __name__ == "__main__":
        talker = Talker()
        talker.spin()


