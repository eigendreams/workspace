#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy

from numpy import array
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import UInt8
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import MultiArrayLayout
from finder.msg import uint8_64

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
        self.databuffer = [chr(0) for x in range(64)]

        self.irPub = rospy.Publisher("ir_out", String)
        self.irSub = rospy.Subscriber("ir_data", uint8_64, self.plotfcn)

        self.bad_flag = False;

    def constrain(self, x, min, max):

        if (x > max):
            return max
        if (x < min):
            return min
        return x

    def plotfcn(self, data):

        self.bad_flag = False;

        for i in range(64):
            """
            if (data.data[i] < -50):
                self.databuffer[i] = 0
                self.bad_flag = True
            elif (data.data[i] > 255 * 16 -50):
                self.databuffer[i] = 0
                self.bad_flag = True
            else:
            """
            self.databuffer[i] = data.data[i]

        self.asccibuffer = "";

        for i in range(64):
            self.asccibuffer += chr(1 * 1)#self.databuffer[i])

            #chr(self.constrain((self.databuffer[i] + 50) / 1, 0, 255))

        #print (str(self.databuffer).strip('[]'))
        #print self.asccibuffer

        if not (self.bad_flag):
            self.irPub.publish(self.asccibuffer);

# Invocación del constructor. Básicamente, si este archivo fuera a ejecutarse como "top level", el
# intérprete le asignaría el "nombre" (protegido) __main__ y ejecutaría las siguientes lineas como
# LO PRIMERO en el archivo (salvo imports y declaraciones)    
if __name__ == '__main__':
    """ main """
    ir_node = IR_Node()
    rospy.spin()
    

