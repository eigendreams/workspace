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
        #
        # el codigo de la stellaris esta dispuesto de manera que si se pierde comunicacion por
        # mas de medio segundo, los motores deben apagarse, sin embrgo, el codigo de la interfaz con la
        # stellaris mismo debe soportar un delay de 2 segundos, antes de forzar el apagado por timeout que
        # pueda suceder por problemas de comunicacion
        #
        self.rate = rospy.get_param("alive_rate", 1)
        #
        # Eventualmente, el valor de alive podria reflejar diferente modos de operacion
        #
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
    