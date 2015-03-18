#!/usr/bin/env python
# -*- coding: utf8 -*-

"""
Esta clase trata de limitar la posibilidad de quemar los motores de DC del robot, para cualquier clase
de robot donde ello sea un posible problema.

Nodo:   motor_output_limit
    Suscriptores:   motorin     Valor de salida PWM al motor dada por el controlador
    Publicadores:   motorout    Valor de salida PWM al motor limitada

Una funcion de ganancia actualiza el valor de salida limite, output_limit, dependiendo del valor de PWM
actual en cada sampleo. Ej, si estamos en salida 0, cada segundo aumenta un 10 la salida maxima al motor,
si es de 100%, cada segundo disminuye un 40 la salida maxima al motor. Con limites fijos entre 0 y 100
"""

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

class Motor_output_limit:

    def __init__(self, node_name_override = 'motor_output_limit'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("motor_output_limit starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)
        self.init_time = self.rospy.get_time()

        self.output_limit_max   = 120;
        self.output_limit       = 100;
        self.output_actual      = 0;
        self.output_command     = 0;

        self.motorout  = rospy.Publisher("motorout", Int16)
        self.motorin   = rospy.Subscriber("motorin", Int16, self.motorcb)

    def gainFcn(self, data):

        return 1 - (abs(data), 0, 100) / 20.

    ###########################################################################
    def map(self, x, in_min, in_max, out_min, out_max):

        return (x - in_min) * (out_max - out_min + 0.) / (in_max - in_min + 0.) + out_min

    def millis(self):

        return int(1000 * (self.rospy.get_time() - self.init_time))

    def constrain(self, x, min, max):

        if (x > max):
            return max
        if (x < min):
            return min
        return x

    def sign(self, a):

        return (a > 0) - (a < 0)
    ###########################################################################

    def motorcb(self, data):

        self.output_command = data.data

        # add punishment and constrain
        self.gain_val = self.gainFcn(self.output_actual)
        self.output_limit = constrain(self.output_limit + self.gain_val, 0, self.output_limit_max)

        # limit the output if outside limits
        if abs(self.output_command) > abs(self.output_limit):
            self.output_actual = self.output_limit * self.sign(self.output_actual)
        else:
            self.output_actual = self.output_command

    def update(self):

        self.motorout.publish(self.output_actual)

    def spin(self):

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

if __name__ == '__main__':

    """ main """
    motor_output_limit = Motor_output_limit()
    motor_output_limit.spin()
