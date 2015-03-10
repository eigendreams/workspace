#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
import Adafruit_BBIO.GPIO as GPIO

class Read_encoders:
    #
    def __init__(self, node_name_override = 'read_encoders'):
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
        self.cs1 = "P9_31"
        self.cs2 = "P9_29"
        self.do  = "P9_25"
        self.clk = "P9_23"
        #
        #
        """
        GPIO.cleanup()
        GPIO.setup(self.cs1, GPIO.OUT)
        GPIO.setup(self.cs2, GPIO.OUT)
        GPIO.setup(self.do,  GPIO.IN)
        GPIO.setup(self.clk, GPIO.OUT)
        # 
        #
        self.closeComm(self.cs1)
        self.closeComm(self.cs2)
        #
        """
        self.e1val = 0
        self.e2val = 0
        #
        # Publicación de la batería como entero
        self.e1Pub = rospy.Publisher("e1", Int16)
        self.e2Pub = rospy.Publisher("e2", Int16)
        #
    def readSingle(self, pincsn):
        #
        self.chainData = 0
        GPIO.output(pincsn,   GPIO.LOW)
        GPIO.output(self.clk, GPIO.LOW)
        for k in range(16):
            GPIO.output(self.clk, GPIO.HIGH)
            self.chainData = (self.chainData << 1) | GPIO.input(self.do)
            GPIO.output(self.clk, GPIO.LOW)
        self.closeComm(pincsn)
        return self.chainData
        #
    def closeComm(self, pincsn):
        #
        GPIO.output(pincsn,   GPIO.HIGH)
        GPIO.output(self.clk, GPIO.HIGH)
        #
    def update(self):
		#
        #self.e1val = self.readSingle(self.cs1)
        #self.e2val = self.readSingle(self.cs2)
        rospy.loginfo("updating") 
        self.e1Pub.publish(self.e1val)
        self.e2Pub.publish(self.e2val)
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
    readencs = Read_encoders()
    readencs.spin()
    