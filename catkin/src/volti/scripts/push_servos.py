#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
import time
import serial
from ino_mod import *
from std_msgs.msg import Int16

class Push_servos:
    #
    def __init__(self, node_name_override = 'push_servos'):
		#
        # El nombre del node deberìa poder especificarse como override desde alguna
        # instancia superior, por si acaso, se establece un default razonable.
        # De todos modos, el nombre del nodo que se guarda "internamente" no puede
        # asumirse igual al proporcionado, por si se invocasen al mismo tiempo varios
        # nodos del mismo "tipo", pues ROS les anexa un número al final como ID
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        # loginfo sirve como medio de DEBUGGING, y para guardar un registro de la
        # ejecución del programa. Por ejemplo, algón overnodo podría hacer uso del log
        # para arreglar errores
        rospy.loginfo("Node starting with name %s", self.nodename) 
        # Quizá algún parámetro superior especifique una frecuencia de salida de datos,
        # por default, si no se encuentra, se usa una de 1 Hz 
        self.rate = float(rospy.get_param("rate", 10))
        #
        self.ser = serial.Serial('/dev/ttyO4', 115200)
        #
        self.m1val = 0
        self.m2val = 0
        self.alval = 0
        # 0x7530 = 30000
        self.datar = [0x75,0x30,0,0,0,0,0,0,0,0]
        #
        self.inittime = rospy.get_time()
        self.timelastal = -1000
        self.timelastnal = -1000
        #
        self.m1sub = rospy.Subscriber("m1", Int16, self.m1cb)
        self.m2sub = rospy.Subscriber("m2", Int16, self.m2cb)
        self.alsub = rospy.Subscriber("al", Int16, self.alcb)
        #
    def m1cb(self,data):
        #
        self.m1val = data.data
        #
    def m2cb(self, data):
        #
        self.m2val = data.data
        #
    def alcb(self, data):
        #
        self.timelastal = millis(self.inittime)
        self.alval = data.data
        #
    def update(self):
		# if al has not been received, shtudown
        if ((millis(self.inittime) - self.timelastal) > 2000):
            self.alval = 0
            self.m1val = 0
            self.m2val = 0
            self.timelastnal = millis(self.inittime)
        # wait a little after each disconnect to avoid jitter
        if ((millis(self.inittime) - self.timelastnal) < 2500):
            self.alval = 0
            self.m1val = 0
            self.m2val = 0
        #
        self.datar[2]  = (self.alval >> 8) & 255
        self.datar[3]  = (self.alval >> 0) & 255
        self.datar[4]  = (self.m1val >> 8) & 255
        self.datar[5]  = (self.m1val >> 0) & 255
        self.datar[6]  = (self.m2val >> 8) & 255
        self.datar[7]  = (self.m2val >> 0) & 255
        self.datar[8]  = ((self.alval + self.m1val + self.m2val) >> 8) & 255
        self.datar[9]  = ((self.alval + self.m1val + self.m2val) >> 0) & 255
        #
        #rospy.loginfo(str(self.datar)) 
        #
        self.ser.write(self.datar)
        #
    def spin(self):
		#
        # rospy.Rate y sleep permiten que el nodo pare su ejecución (cuidado, los subscriptores
        # NO están restringidos por el resto del nodo más allá de la función de callback) e 
        # intente ejecutarse cada tantos ms, dados por el rate declarado en el constructor de la
        # clase. Ahorra recursos
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
        #
# Invocación del constructor. Básicamente, si este archivo fuera a ejecutarse como "top level", el
# intérprete le asignaría el "nombre" (protegido) __main__ y ejecutaría las siguientes lineas como
# LO PRIMERO en el archivo (salvo imports y declaraciones)    
if __name__ == '__main__':
    """ main """
    push_servos = Push_servos()
    push_servos.spin()
    