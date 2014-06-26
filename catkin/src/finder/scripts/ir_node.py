#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy

from numpy import array
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import MultiArrayLayout

class IR_Node():
    
    def __init__(self, node_name_override = 'ir_node'):

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
        rospy.loginfo("ir_node starting with name %s", self.nodename) 
        # Quizá algún parámetro superior especifique una frecuencia de salida de datos,
        # por default, si no se encuentra, se usa una de 1 Hz 
        self.rate = rospy.get_param("ir_node_rate", 10)
        # Publicación de la batería como entero
        self.databuffer = [0 for x in range(64)]

        self.irPub = rospy.Publisher("ir_out", String)
        self.irSub = rospy.Subscriber("ws", Int16MultiArray, self.plotfcn)

    def plotfcn(self, data):

        for i in range(64):
            self.databuffer[i] = (data.data[i] + 50) / 16

        self.asccibuffer = "";

        for i in range(64):
            self.asccibuffer += chr(self.databuffer[i])

        print (str(self.databuffer).strip('[]'))
        #print self.asccibuffer

        self.irPub.publish(self.asccibuffer);

# Invocación del constructor. Básicamente, si este archivo fuera a ejecutarse como "top level", el
# intérprete le asignaría el "nombre" (protegido) __main__ y ejecutaría las siguientes lineas como
# LO PRIMERO en el archivo (salvo imports y declaraciones)    
if __name__ == '__main__':
    """ main """
    ir_node = IR_Node()
    rospy.spin()
    
