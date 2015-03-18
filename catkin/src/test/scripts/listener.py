#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class Listener:
        def __init__(self, node_name = 'listener'):
                rospy.init_node(node_name)
                self.nodename = rospy.get_name()
                rospy.loginfo("Started node %s", self.nodename)
                self.sub = rospy.Subscriber("chatter", String, self.callback())
        def callback(self, data):
                rospy.loginfo(rospy.get_caller_id() + "I heard %s" + data.data)
if __name__ == "__main__":
        listener = Listener()
        rospy.spin()
