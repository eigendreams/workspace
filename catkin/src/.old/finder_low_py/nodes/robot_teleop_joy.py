#!/usr/bin/env python
# -*- coding: utf8 -*-

# importados SIEMPRE
import rospy
import roslib
roslib.load_manifest('finder_low_py')

# imports para el manejo de arrays numéricos
from numpy import array
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import MultiArrayLayout

# Imports de funciones matemáticas
from math import exp

# Joy por Joystick
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class Robot_teleop_joy:
    
    def __init__(self, node_name_in_joy = 'robot_teleop_joy'):
        # El nombre del node deberìa poder especificarse como override desde alguna
        # instancia superior, por si acaso, se establece un default razonable.
        # De todos modos, el nombre del nodo que se guarda "internamente" no puede
        # asumirse igual al proporcionado, por si se invocasen al mismo tiempo varios
        # nodos del mismo "tipo", pues ROS les anexa un número al final como ID
        rospy.init_node(node_name_in_joy)
        self.nodename = rospy.get_name()
        # loginfo sirve como medio de DEBUGGING, y para guardar un registro de la
        # ejecución del programa. Por ejemplo, algón overnodo podría hacer uso del log
        # para arreglar errores
        rospy.loginfo("%s starting" % self.nodename) 
        # QUizá algún parámetro superior especifique una frecuencia de salida de datos,
        # por default, si no se encuentra, se usa una de 20 Hz 
        self.rate = rospy.get_param("robot_teleop_joy_rate", 2)
        # Conexión con Joystick, dos diccionarios especifican los botones de interés.
        # Estos datos TAMBIÉN podrían ser pedidos como parámetros. Finalmente, nos
        # suscribimos al nodo joy de TIPO Joy (son fijos, a menos que se invocaran
        # varios joysticks, pero la versión estándar de Joy de todos modos no
        # lo permite). El nodo Joy al publicar llama a la función joyCb
        self.axes_names = {'linear':5, 'angular':0}
        self.buttons_names = {'reverse':1, 'start':8, 'stop':2}
        self.joySub = rospy.Subscriber("joy", Joy, self.joyCb)
        # Hook con el node de recepción de Arduino. Puesto que el que está en el uC
        # es un subscriptor, aquí debemos publicar el tópico, llamado vdes. Las
        # varias declaraciones usadas son NECESARIAS para usar MultiArrays en rospy.
        # Los datos a publicar se ponen en data_vdes, pero podría ser algún otro 
        # arreglo, que cumpla con las especificaciones de dimension dadas en
        # vals_vdes y layout_vdes, FIJAS en el uC
        self.vals_vdes = MultiArrayDimension()
        self.layout_vdes = MultiArrayLayout()
        self.data_vdes = []
        self.vals_vdes.label = "vdes"
        self.vals_vdes.size = 4
        self.vals_vdes.stride = 4
        self.layout_vdes.data_offset = 4
        self.layout_vdes.dim.append(self.vals_vdes)
        self.vdesPub = rospy.Publisher('vdes', Int16MultiArray)
        # Otró tópic en Arduino especifíca el "modo" de control. Si true, entonces
        # los datos publicados en vdes se pasan DIRECTAMENTE a las salidas PWM
        self.McPub = rospy.Publisher("Mc", Bool)
        self.MrPub = rospy.Publisher("Mr", Bool)
        # Variables internas
        self.reverse = False
        self.mult_reverse = 0
        self.angular_rate = 0
        self.linear_rate = 0
        # lista de elementos de filtrado
        self.exp_term_angular_rate = 0
        self.exp_term_lin_prod = 0   
        self.last_lin_prod = 0
        self.last_angular_rate = 0
        # log output
        rospy.loginfo("%s initialized" % self.nodename) 
               
        
    def joyCb(self, data):
        # data catch. Los métodos de callback deberían ser cortos y 
        # eficientes, además de imposibles de "bloquear"
        self.reverse = data.buttons[self.buttons_names['reverse']]
        self.angular_rate = data.axes[self.axes_names['angular']]
        self.linear_rate = (1 - data.axes[self.axes_names['linear']]) / 2.
        
        # Activación y desactivación del relé de paso de potencia del
        # robot, quizá el uso de locks haga esto más eficinete al
        # evitar publicar varias veces el mismo valor
        if (data.buttons[self.buttons_names['start']]):
            self.MrPub.publish(True)
        if (data.buttons[self.buttons_names['stop']]):
            self.MrPub.publish(False)
            
        
        
    def update(self):
        # una variable registra el signo de la reversa
        if (self.reverse):
            self.mult_reverse = -1.
        else:
            self.mult_reverse = 1.
        
        self.lin_prod = self.mult_reverse * self.linear_rate
        
        #rospy.loginfo("%s lin_prod" % self.lin_prod) 
        #rospy.loginfo("%s ang_rate" % self.angular_rate)
        
        # Etapa de filtrado con decaimiento exponencial
        self.now_dif_lin_prod = self.last_lin_prod - self.lin_prod
        self.exp_term_lin_prod = (self.exp_term_lin_prod + self.now_dif_lin_prod) * exp(-3. * (1. / self.rate))
        self.out_lin_prod = self.lin_prod + self.exp_term_lin_prod
        self.last_lin_prod = self.lin_prod
        
        self.now_dif_angular_rate = self.last_angular_rate - self.angular_rate
        self.exp_term_angular_rate = (self.exp_term_angular_rate + self.now_dif_angular_rate) * exp(-3. * (1. / self.rate))
        self.out_angular_rate = self.angular_rate + self.exp_term_angular_rate
        self.last_angular_rate = self.angular_rate
        
        # Se determinan los valores finales de cada rueda
        self.m_right = (self.out_lin_prod + self.out_angular_rate / 2. + 1.5) * 85
        self.m_left = (self.out_lin_prod - self.out_angular_rate / 2. + 1.5) * 85
        
        # en base a los valores anteriores calculamos la velocidad deseada de cada uno de los
        # motores, en particular, los motores anteriores 1 y 2 deben manejarse de forma diferente
        # a los motores delatneros 3 y 4, el motor 1 y 4 están del lado izquierdo, y el motor
        # 2 y 3 están del lado derecho. El código de arduino replica directamente, si se usa en
        # PWM, los valores aquí espuestos es el orden [1, 2, 3, 4] sin hacer correcciones de
        # sentido o intentar hacer control Además, por la limitaciones del método de sensado
        # de la velocidad, es (prácticamente) imposible hacer control a bajas velocidades
        
        """
        Sin embargo, esto no es suficiente para lograr el giro en una u otra dirección. Básicamente,
        si el angular rate fuera muy positivo, deberiamos girar a la izquierda, y por ende la rueda trasera
        de la izquierda (1) debería estar parada. 
        
        De la misma forma, si quisieramos girar a la derecha porque
        el angular rate es muy negativo, la reuda anterior derecha (2) debería estar apagada. 
        
        Como no hay relación directa entre el angular rate y los valores de salida PWM en forma de que no 
        están limitados solo podemos confiar básicamente en el valor de la resta:
        
        m_right - m_left ~= angular_rate corregido
        
        A medida que la cantidad anterior crece (de -255 a 255) el angular rate es más positivo, por lo que
        deseamos girar a la izquierda, y por ende debemos apagar la rueda trasera de la izquierda (1).
        
        Inversamente, si la cantidad decrece, el angular rate es más negativo, por lo que queremos girar
        a la derecha, por lo que debemos apagar el motor trase de la derecha (2)
        
        Así que dependiendo del signo:
        
        self.resta = self.m_right - self.m_left
        if (self.resta > 0) :    #apagar el motor 1, rueda trasera de la izquierda
            self.v_mot1 = self.v_mot1 * (255 - resta) / resta
        else :
            self.v_mot2 = self.v_mot2 * (255 + resta) / resta
        
        """
        
        """
        self.resta = self.m_right - self.m_left
        if (self.resta > 0) :    #apagar el motor 1, rueda trasera de la izquierda
            self.v_mot1 = self.v_mot1 * (255 - self.resta) / self.resta   #resta va de 0 a 255
        else :
            self.v_mot2 = self.v_mot2 * (255 + self.resta) / (-1 * self.resta)   #resta va de -0 a -255
        """
        
        self.v_mot1 = int(self.m_left)  #rueda anterior izquierda
        self.v_mot2 = int(self.m_right) #rueda anterior derecha
        self.v_mot3 = int(self.m_right) #rueda delantera derecha
        self.v_mot4 = int(self.m_left)  #rueda delantera izquierda
        
        # self.v_mot1 = int(self.m_left)  #rueda anterior izquierda
        # self.v_mot2 = int(self.m_right) #rueda anterior derecha
        # resta = v_mot2 - v_mot1
        
        self.resta = self.m_right - self.m_left
        if (self.resta > 0) :
            self.v_mot2 = int(127 + (self.v_mot2 - 127) * (85 - self.resta) / 85)   #resta va de 0 a 255
            self.v_mot1 = self.v_mot3
        else :
            self.v_mot1 = int(127 + (self.v_mot1 - 127) * (85 + self.resta) / 85)   #resta va de -0 a -255
            self.v_mot2 = self.v_mot4
        
        # rospy.loginfo("%s right" % self.m_right) 
        # rospy.loginfo("%s left" % self.m_left)   
        # rospy.loginfo("%s resta" % self.resta) 
        
        # la publicación es simple, los datos se "empaquetan" en un array, y a través del 
        # publicador como fuera definido en el constructor de la clase, publicamos dichos
        # datos Y el layout que les describe en "forma"
        self.data_vdes = [self.v_mot1, self.v_mot2 , self.v_mot3, self.v_mot4]
        self.vdesPub.publish(self.layout_vdes, self.data_vdes)
        
        
    def initNodes(self):
        # Nos aseguramos que el modo de control del uC (si los datos en vdes se entienden como
        # valores de PWM o de velocidad deseada) sea el correcto. Actualmente se manejan PWMs
        self.McPub.publish(True)
        

    def spin(self):
        # rospy.Rate y sleep permiten que el nodo pare su ejecución (cuidado, los subscriptores
        # NO están restringidos por el resto del nodo más allá de la función de callback) e 
        # intente ejecutarse cada tantos ms, dados por el rate declarado en el constructor de la
        # clase. Ahorra recursos
        self.initNodes()
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

# Invocación del constructor. Básicamente, si este archivo fuera a ejecutarse como "top level", el
# intérprete le asignaría el "nombre" (protegido) __main__ y ejecutaría las siguientes lineas como
# LO PRIMERO en el archivo (salvo imports y declaraciones)    
if __name__ == '__main__':
    """ main """
    robot_teleop_joy = Robot_teleop_joy()
    robot_teleop_joy.spin()
    

