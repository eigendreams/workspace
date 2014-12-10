#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16

class Interface_Servo:
    
    def __init__(self, node_name_override = 'interface_servo'):
        
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
        rospy.loginfo("interface_servo starting with name %s", self.nodename) 

        # QUizá algún parámetro superior especifique una frecuencia de salida de datos,
        # por default, si no se encuentra, se usa una de 20 Hz 
        self.freq = 5
        self.rate = rospy.get_param("interface_servo_rate", self.freq)

        # Conexión con Servostick, dos diccionarios especifican los botones de interés.
        # Estos datos TAMBIÉN podrían ser pedidos como parámetros. Finalmente, nos
        # suscribimos al nodo servo de TIPO Servo (son fijos, a menos que se invocaran
        # varios servosticks, pero la versión estándar de Servo de todos modos no
        # lo permite). El nodo Servo al publicar llama a la función servoCb
        self.bolitaSub = rospy.Subscriber("bolita", Int16, self.bolitaCb)
        self.servo1Pub = rospy.Publisher("servo1", Int16)

        self.time_lock = 0
        self.time_max  = 5000   # valor en ms
        self.servo_val = -90
        self.servo_max = 90     # servo_max es la posicion "maxima" con la que sale la bolita
                                # servo_val y servo_max DEBEN tener signos opuestos!!!

    def bolitaCb(self, data):
		if (data.data > 0):
			if (self.time_lock > 0):
				# Estamos DENTRO de una operacion de movimiento de bolita!!!
				# debemos esperar a que termine el proceso y no hacer nada!!!
				return 0

			# reiniciando el contador del proceso al time max
			self.time_lock = self.time_max
        
    def update(self):

        # decrementando el valor de tiempo transcurrido del proceso
        if (self.time_lock > 0):
            self.time_lock = self.time_lock - 1000. / self.freq
        else:
            self.time_lock = 0

        # calculando el valor de angulo deseado para el servo de la bolita
	self.servo_val = self.servo_max - 2 * self.servo_max * abs(self.time_lock - self.time_max / 2.0) / (self.time_max / 2.0)
	self.servo1Pub.publish(self.servo_val)
        
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
    interface_servo = Interface_Servo()
    interface_servo.spin()
    

