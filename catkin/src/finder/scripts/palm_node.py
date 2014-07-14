#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

class Palm_node:

    def __init__(self, node_name_override = 'palm_node'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("palm_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        self.palm_out = 0
        self.palm_ang = 0
        self.palm_vel = 0

        self.palmOutPub = rospy.Publisher("palm_out", Int16)
        self.palmAngPub = rospy.Publisher("palm_ang", Float32)
        self.palmVelPub = rospy.Publisher("palm_vel", Float32)
        self.palmResetSub = rospy.Subscriber("palm_reset", Int16, self.palmResetCb)
        self.palmDesSub = rospy.Subscriber("palm_des", Float32, self.palmDesCb)


    def millis(self):

        return int(1000 * (self.rospy.get_time() - self.init_time))


    def constrain(self, x, min, max):

        if (x > max):
            return max
        if (x < min):
            return min
        return x


    def palmResetCb(self, data):
        
        if (data.data == 1):
            self.palm_out = 0


    def palmDesCb(self, data):

        self.palm_vel = data.data


    def update(self):

        if (abs(self.palm_vel) < 0.1):
            # Mantiene posicion constante
            self.palm_out = self.palm_out
        else:
            # cambia segun la velocidad y restringe a rango
            self.palm_out += self.palm_vel * 200. / 3.1416
            self.palm_out = self.constrain(self.palm_out, -100, 100)

        # coversion a radianes
        self.palm_ang = self.palm_out * 3.1416 / 200.

        self.palmOutPub.publish(int(self.palm_out))
        self.palmAngPub.publish(self.palm_ang)
        self.palmVelPub.publish(self.palm_vel)


    def spin(self):

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':

    """ main """
    palm_node = Palm_node()
    palm_node.spin() 


"""
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
"""