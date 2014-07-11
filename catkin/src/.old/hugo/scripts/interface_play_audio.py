#!/usr/bin/env python
# -*- coding: utf8 -*-

import subprocess
import rospy
from std_msgs.msg import Int16

class Interface_Play_Audio:
    
    def __init__(self, node_name_override = 'interface_play_audio'):
        
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
        rospy.loginfo("interface_play_audio starting with name %s", self.nodename) 

        # QUizá algún parámetro superior especifique una frecuencia de salida de datos,
        # por default, si no se encuentra, se usa una de 20 Hz 
        self.freq = 5
        self.rate = rospy.get_param("interface_play_audio_rate", self.freq)

        # Conexión con Playstick, dos diccionarios especifican los botones de interés.
        # Estos datos TAMBIÉN podrían ser pedidos como parámetros. Finalmente, nos
        # suscribimos al nodo play de TIPO Play (son fijos, a menos que se invocaran
        # varios playsticks, pero la versión estándar de Play de todos modos no
        # lo permite). El nodo Play al publicar llama a la función playCb
        self.audioSub = rospy.Subscriber("audio", Int16, self.audioCb)

        self.time_lock = 0
        self.time_max  = 10000   # valor en ms

    def audioCb(self, data):
        if (data.data > 0):
            if (self.time_lock > 0):
                # Estamos DENTRO de una operacion de movimiento de bolita!!!
                # debemos esperar a que termine el proceso y no hacer nada!!!
                return 0

            # reiniciando el contador del proceso al time max
            self.time_lock = self.time_max
            subprocess.call(["aplay", '/home/pi/audio/audio' + str(data.data) + '.wav' ])
        
    def update(self):

        # decrementando el valor de tiempo transcurrido del proceso
        if (self.time_lock > 0):
            self.time_lock = self.time_lock - 1000. / self.freq
        else:
            self.time_lock = 0
        
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
    interface_play_audio = Interface_Play_Audio()
    interface_play_audio.spin()
    
