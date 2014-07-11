#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16

class Interface_Motor:
    
    def __init__(self, node_name_override = 'interface_motor'):
        
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
        rospy.loginfo("interface_motor starting with name %s", self.nodename) 

        # QUizá algún parámetro superior especifique una frecuencia de salida de datos,
        # por default, si no se encuentra, se usa una de 20 Hz 
        self.rate = rospy.get_param("interface_motor_rate", 5)

        # Conexión con Joystick, dos diccionarios especifican los botones de interés.
        # Estos datos TAMBIÉN podrían ser pedidos como parámetros. Finalmente, nos
        # suscribimos al nodo joy de TIPO Joy (son fijos, a menos que se invocaran
        # varios joysticks, pero la versión estándar de Joy de todos modos no
        # lo permite). El nodo Joy al publicar llama a la función joyCb
        self.leftSub = rospy.Subscriber("left", Int16, self.leftCb)
        self.rightSub = rospy.Subscriber("right", Int16, self.rightCb)
        self.motor1Pub = rospy.Publisher("motor1", Int16)
        self.motor2Pub = rospy.Publisher("motor2", Int16)

        self.motor1val = 0
        self.motor2val = 0

        self.timeLastMsg = 0
        
    def leftCb(self, data):
        self.motor1val = data.data
        self.timeLastMsg = rospy.get_time()

    def rightCb(self, data):
        self.motor2val = data.data
        self.timeLastMsg = rospy.get_time()
        
    def update(self):
        if (rospy.get_time() - self.timeLastMsg > 5):
            # We timedOut, STOP the motors!!!!!
            self.motor1val = 0
            self.motor2val = 0
        
        self.motor1Pub.publish(self.motor1val)
        self.motor2Pub.publish(self.motor2val)
        
    def spin(self):
        
        # rospy.Rate y sleep permiten que el nodo pare su ejecución (cuidado, los subscriptores
        # NO están restringidos por el resto del nodo más allá de la función de callback) e 
        # intente ejecutarse cada tantos ms, dados por el rate declarado en el constructor de la
        # clase. Ahorra recursos
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

# Invocación del constructor. Básicamente, si este archivo fuera a ejecutarse como "top level", el
# intérprete le asignaría el "nombre" (protegido) __main__ y ejecutaría las siguientes lineas como
# LO PRIMERO en el archivo (salvo imports y declaraciones)    
if __name__ == '__main__':
    """ main """
    interface_motor = Interface_Motor()
    interface_motor.spin()
    

