#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

class Interface_Joy:
    
    def __init__(self, node_name_override = 'interface_joy'):
        
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
        rospy.loginfo("interface_joy starting with name %s", self.nodename) 

        # QUizá algún parámetro superior especifique una frecuencia de salida de datos,
        # por default, si no se encuentra, se usa una de 20 Hz 
        self.freq = 5 
        self.rate = rospy.get_param("interface_joy_rate", self.freq)

        # Conexión con Joystick, dos diccionarios especifican los botones de interés.
        # Estos datos TAMBIÉN podrían ser pedidos como parámetros. Finalmente, nos
        # suscribimos al nodo joy de TIPO Joy (son fijos, a menos que se invocaran
        # varios joysticks, pero la versión estándar de Joy de todos modos no
        # lo permite). El nodo Joy al publicar llama a la función joyCb

        # El eje horizontal de los stick y la cruz tiene signos "invertidos", los triggers inician en 1 y
        # avanxan hacia -1 al ser presionados, 
        self.axes_names = {'left_stick_hor':0, 'left_stick_ver':1, 'left_trig':2, 'right_stick_hor':3, 'right_stick_ver':4, 'right_trig':5, 'cross_hor':6, 'cross_ver':7}
        self.buttons_names = {'A':0, 'B':1, 'X':2, 'Y':3, 'left_btn':4, 'right_btn':5, 'back':6, 'start':7, 'cross':8, 'left_stick':9, 'right_stick':10}
        self.joySub = rospy.Subscriber("joy", Joy, self.joyCb)

        self.audioPub = rospy.Publisher("audio", Int16)
        self.bolitaPub = rospy.Publisher("bolita", Int16)
        self.videoPub  = rospy.Publisher("video" , Int16)
        self.leftPub   = rospy.Publisher("left"  , Int16) # CONTINUO!
        self.rightPub  = rospy.Publisher("right" , Int16) # CONTINUO!

        self.left_lock = 0
        self.right_lock = 0
        self.audio_lock = 0
        self.bolita_lock = 0
        self.video_lock = 0
        
    def joyCb(self, data):
        
        # data catch. Los métodos de callback deberían ser cortos y 
        # eficientes, además de imposibles de "bloquear"
        self.left_lock = int(data.axes[self.axes_names['left_stick_ver']] * 100)
        self.right_lock = int(data.axes[self.axes_names['right_stick_ver']] * 100)

        # Seleccionando la pista de audio
        self.audio_lock = 0
        if (data.buttons[self.buttons_names['A']]):
            self.audio_lock = 1
        if (data.buttons[self.buttons_names['B']]):
            self.audio_lock = 2
        if (data.buttons[self.buttons_names['X']]):
            self.audio_lock = 3
        if (data.buttons[self.buttons_names['Y']]):
            self.audio_lock = 4

        self.bolita_lock = data.buttons[self.buttons_names['left_btn']]
        self.video_lock  = data.buttons[self.buttons_names['right_btn']]
        
    def update(self):

        self.audioPub.publish(self.audio_lock)
        self.bolitaPub.publish(self.bolita_lock)
        self.videoPub.publish(self.video_lock)
        self.leftPub.publish(self.left_lock)
        self.rightPub.publish(self.right_lock)
        
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
    interface_joy = Interface_Joy()
    interface_joy.spin()
    

