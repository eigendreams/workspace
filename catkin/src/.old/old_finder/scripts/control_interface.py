#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from finder.msg import TwoWheelInt16
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import Int16


# Imports de funciones matemáticas
from math import exp

class Finder_diff_joy:
    
    def __init__(self, node_name_override = 'finder_diff_joy'):
		
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
        rospy.loginfo("finder_diff_joy starting with name %s", self.nodename) 
        # QUizá algún parámetro superior especifique una frecuencia de salida de datos,
        # por default, si no se encuentra, se usa una de 20 Hz 
        self.rate = rospy.get_param("finder_diff_joy_rate", 20)
        # Conexión con Joystick, dos diccionarios especifican los botones de interés.
        # Estos datos TAMBIÉN podrían ser pedidos como parámetros. Finalmente, nos
        # suscribimos al nodo joy de TIPO Joy (son fijos, a menos que se invocaran
        # varios joysticks, pero la versión estándar de Joy de todos modos no
        # lo permite). El nodo Joy al publicar llama a la función joyCb
        self.axes_names = {'h_left_pad':0, 'v_left_pad':1, 'v_right_pad':4, 'h_right_pad':3, 'h_arrow': 6,
                            'left_trigger_pad': 2, 'right_trigger_pad': 5}
        self.buttons_names = {'reverse':1, 'left_trigger_button': 4, 'right_trigger_button': 5}
        self.joySub = rospy.Subscriber("joy", Joy, self.joyCb)
        
        # Setting up the output data type
        # self.wheelData = TwoWheelInt16()
        # self.wheelData.left = 0
        # self.wheelData.right = 0
        # self.wheelPub = rospy.Publisher("vdes", TwoWheelInt16) 

        #Publishers for the motors
        self.motorArmBasePub = rospy.Publisher("motor_arm_base", Int16)
<<<<<<< HEAD
=======

>>>>>>> 8314afc5dc479249d3fb14b3b34d9e8cdf6bf26c
        self.motorArm1Pub = rospy.Publisher("motor_arm_1", Int16)
        self.motorArm2Pub = rospy.Publisher("motor_arm_2", Int16)
        self.motorArm3pub = rospy.Publisher("servo_arm_1", Int16)
        self.motorArm4pub = rospy.Publisher("servo_arm_2", Int16)
        self.servoArmGripperPub = rospy.Publisher("servo_arm_gripper", Int16)
<<<<<<< HEAD
        self.motorTractionLeftPub = rospy.Publisher("motor_traction_left", Int16)
        self.motorTractionRightPub = rospy.Publisher("motor_traction_right", Int16)
=======

        self.motorTractionLeftPub = rospy.Publisher("motor_traction_left", Int16)
        self.motorTractionRightPub = rospy.Publisher("motor_traction_right", Int16)

>>>>>>> 8314afc5dc479249d3fb14b3b34d9e8cdf6bf26c
        self.motorTractionArmFrPub = rospy.Publisher("motor_traction_arm_fr", Int16)
        self.motorTractionArmFlPub = rospy.Publisher("motor_traction_arm_fl", Int16)
        self.motorTractionArmBrPub = rospy.Publisher("motor_traction_arm_br", Int16)
        self.motorTractionArmBlPub = rospy.Publisher("motor_traction_arm_bl", Int16)
        
        # Otro topic en Arduino especifíca el "modo" de control. Si true, entonces
        # los datos publicados en vdes se pasan DIRECTAMENTE a las salidas PWM
        # self.McPub = rospy.Publisher("mc", Bool)
        
        # Variables internas
        self.angular_rate = 0
        self.linear_rate = 0
        self.motor_arm_base_rate = 0
        self.motor_traction_arm_bl_active = 0
        self.motor_traction_arm_fr_active = 0
        self.motor_traction_arm_br_active = 0
        self.motor_traction_arm_fl_active = 0
        self.motor_traction_arm_rate = 0
        self.motor_traction_arm_fr_rate = 0
        self.motor_traction_arm_fl_rate = 0
        self.motor_traction_arm_br_rate = 0
        self.motor_traction_arm_bl_rate = 0
        # lista de elementos de filtrado
        # self.exp_term_angular_rate = 0
        # self.exp_term_lin_prod = 0   
        # self.last_lin_prod = 0
        # self.last_angular_rate = 0               
        
    def joyCb(self, data):
		
        # data catch. Los métodos de callback deberían ser cortos y 
        # eficientes, además de imposibles de "bloquear"
        # self.reverse = data.buttons[self.buttons_names['reverse']]
        self.angular_rate = data.axes[self.axes_names['h_left_pad']] * 100
        self.linear_rate = data.axes[self.axes_names['v_left_pad']] * 100
<<<<<<< HEAD
        self.motor_arm_base_rate = data.axes[self.axes_names['h_arrow']] * -50
=======

        self.motor_arm_base_rate += data.axes[self.axes_names['h_arrow']] * -10

>>>>>>> 8314afc5dc479249d3fb14b3b34d9e8cdf6bf26c
        self.motor_traction_arm_bl_active = data.buttons[self.buttons_names['left_trigger_button']]
        self.motor_traction_arm_fr_active = data.axes[self.axes_names['right_trigger_pad']] - 1
        self.motor_traction_arm_br_active = data.buttons[self.buttons_names['right_trigger_button']]
        self.motor_traction_arm_fl_active = data.axes[self.axes_names['left_trigger_pad']] - 1 

        self.motor_traction_arm_rate = data.axes[self.axes_names['v_right_pad']] * 100
        
    def motorTractionUpdate(self):
		
        # una variable registra el signo de la reversa
        # if (self.reverse):
        #     self.mult_reverse = -1.
        # else:
        #     self.mult_reverse = 1.
        
        # self.lin_prod = self.mult_reverse * self.linear_rate
        
        #rospy.loginfo("%s lin_prod" % self.lin_prod) 
        #rospy.loginfo("%s ang_rate" % self.angular_rate)
        
        # Etapa de filtrado con decaimiento exponencial
        # self.now_dif_lin_prod = self.last_lin_prod - self.lin_prod
        # self.exp_term_lin_prod = (self.exp_term_lin_prod + self.now_dif_lin_prod) * exp(-3. * (1. / self.rate))
        # self.out_lin_prod = self.lin_prod + self.exp_term_lin_prod
        # self.last_lin_prod = self.lin_prod
        
        # self.now_dif_angular_rate = self.last_angular_rate - self.angular_rate
        # self.exp_term_angular_rate = (self.exp_term_angular_rate + self.now_dif_angular_rate) * exp(-3. * (1. / self.rate))
        # self.out_angular_rate = self.angular_rate + self.exp_term_angular_rate
        # self.last_angular_rate = self.angular_rate
        
        # # Se determinan los valores finales de cada rueda
        # self.m_right = (self.out_lin_prod + self.out_angular_rate / 2. + 1.5) * 85
        # self.m_left = (self.out_lin_prod - self.out_angular_rate / 2. + 1.5) * 85
        
        # # rospy.loginfo("Finder m_right: %s m_left: %s", self.m_right, self.m_left) 
        
        # self.wheelData.right = self.m_right
        # self.wheelData.left = self.m_left
        
        motor_left = self.linear_rate + self.angular_rate
        motor_right = self.linear_rate - self.angular_rate

        motor_left = self.constrain(motor_left,-100,100)
        motor_right = self.constrain(motor_right,-100,100)

        # self.wheelPub.publish(self.wh)
        # self.
        self.motorTractionLeftPub.publish(motor_left)
        self.motorTractionRightPub.publish(motor_right)

<<<<<<< HEAD
    def motorArmUpdate(self):

        self.motorArmBasePub.publish(self.motor_arm_base_rate)
=======
        print "left: " + str(motor_left)
        print "right: " + str(motor_right)

    def motorArmUpdate(self):

        self.motorArmBasePub.publish(self.motor_arm_base_rate)
        print "base: " + str(self.motor_arm_base_rate)
>>>>>>> 8314afc5dc479249d3fb14b3b34d9e8cdf6bf26c

    def motorTractionArmUpdate(self):

        # motor_traction_arm_fr_rate = 0
        # motor_traction_arm_fl_rate = 0
        # motor_traction_arm_br_rate = 0
        # motor_traction_arm_bl_rate = 0

        if self.motor_traction_arm_fr_active:
<<<<<<< HEAD
            self.motor_traction_arm_fr_rate = self.motor_traction_arm_rate
        if self.motor_traction_arm_fl_active:
            self.motor_traction_arm_fl_rate = self.motor_traction_arm_rate
        if self.motor_traction_arm_br_active:
            self.motor_traction_arm_br_rate = self.motor_traction_arm_rate
        if self.motor_traction_arm_bl_active:
            self.motor_traction_arm_bl_rate = self.motor_traction_arm_rate
=======
            self.motor_traction_arm_fr_rate += self.motor_traction_arm_rate / 50.
        if self.motor_traction_arm_fl_active:
            self.motor_traction_arm_fl_rate += self.motor_traction_arm_rate / 50.
        if self.motor_traction_arm_br_active:
            self.motor_traction_arm_br_rate += self.motor_traction_arm_rate / 50.
        if self.motor_traction_arm_bl_active:
            self.motor_traction_arm_bl_rate += self.motor_traction_arm_rate / 50.

        self.motor_traction_arm_fr_rate = self.constrain(self.motor_traction_arm_fr_rate, -100, 100);
        self.motor_traction_arm_fl_rate = self.constrain(self.motor_traction_arm_fl_rate, -100, 100);
        self.motor_traction_arm_br_rate = self.constrain(self.motor_traction_arm_br_rate, -100, 100);
        self.motor_traction_arm_bl_rate = self.constrain(self.motor_traction_arm_bl_rate, -100, 100);                                                 
>>>>>>> 8314afc5dc479249d3fb14b3b34d9e8cdf6bf26c

        self.motorTractionArmFrPub.publish(self.motor_traction_arm_fr_rate)
        self.motorTractionArmFlPub.publish(self.motor_traction_arm_fl_rate)
        self.motorTractionArmBrPub.publish(self.motor_traction_arm_br_rate)
        self.motorTractionArmBlPub.publish(self.motor_traction_arm_bl_rate)

<<<<<<< HEAD
=======
        print "fr: " + str(self.motor_traction_arm_fr_rate)
        print "fl: " + str(self.motor_traction_arm_fl_rate)
        print "br: " + str(self.motor_traction_arm_br_rate)
        print "bl: " + str(self.motor_traction_arm_bl_rate)
>>>>>>> 8314afc5dc479249d3fb14b3b34d9e8cdf6bf26c
        
    def initNodes(self):
		
        # Nos aseguramos que el modo de control del uC (lsi los datos en vdes se entienden como
        # valores de PWM o de velocidad deseada) sea el correcto. Actualmente se manejan PWMs
        # self.McPub.publish(False)
        pass
        
    def spin(self):
		
        # rospy.Rate y sleep permiten que el nodo pare su ejecución (cuidado, los subscriptores
        # NO están restringidos por el resto del nodo más allá de la función de callback) e 
        # intente ejecutarse cada tantos ms, dados por el rate declarado en el constructor de la
        # clase. Ahorra recursos
        self.initNodes()
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # self.update()
            self.motorTractionUpdate()
            self.motorArmUpdate()
            self.motorTractionArmUpdate()
            r.sleep()

    def constrain(self, value, lower_limit, higher_limit):
        
        if value > higher_limit:
            value = higher_limit
        elif value < lower_limit:
            value = lower_limit
<<<<<<< HEAD

=======
>>>>>>> 8314afc5dc479249d3fb14b3b34d9e8cdf6bf26c
        return value

# Invocación del constructor. Básicamente, si este archivo fuera a ejecutarse como "top level", el
# intérprete le asignaría el "nombre" (protegido) __main__ y ejecutaría las siguientes lineas como
# LO PRIMERO en el archivo (salvo imports y declaraciones)    
if __name__ == '__main__':
    """ main """
    finder_diff_joy = Finder_diff_joy()
    finder_diff_joy.spin()
    

