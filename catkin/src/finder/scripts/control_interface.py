#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Float32


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
        self.buttons_names = {'reverse':1, 'left_trigger_button': 4, 'right_trigger_button': 5, 'right_pad_button': 10}
        
        
        # Setting up the output data type
        # self.wheelData = TwoWheelInt16()
        # self.wheelData.left = 0
        # self.wheelData.right = 0
        # self.wheelPub = rospy.Publisher("vdes", TwoWheelInt16) 

        #Publishers for the motors
        self.basePub = rospy.Publisher("base_des", Float32)

        self.armPub = rospy.Publisher("arm_des", Float32)
        self.forearmPub = rospy.Publisher("forearm_des", Float32)
        self.gripperPub = rospy.Publisher("gripper_des", Float32)
        self.wristPub = rospy.Publisher("wrist_des", Float32)
        self.palmPub = rospy.Publisher("palm_des", Float32)

        self.leftPub = rospy.Publisher("left_des", Float32)
        self.rightPub = rospy.Publisher("right_des", Float32)

        self.frPub = rospy.Publisher("fr_des", Float32)
        self.flPub = rospy.Publisher("fl_des", Float32)
        self.brPub = rospy.Publisher("br_des", Float32)
        self.blPub = rospy.Publisher("bl_des", Float32)

        self.frResetPub = rospy.Publisher("fr_reset", Int16)
        self.flResetPub = rospy.Publisher("fl_reset", Int16)
        self.brResetPub = rospy.Publisher("br_reset", Int16)
        self.blResetPub = rospy.Publisher("bl_reset", Int16)
        
        # Otro topic en Arduino especifíca el "modo" de control. Si true, entonces
        # los datos publicados en vdes se pasan DIRECTAMENTE a las salidas PWM
        # self.McPub = rospy.Publisher("mc", Bool)
        
        # Variables internas
        self.angular_rate = 0
        self.linear_rate = 0
        self.base_rate = 0

        # Activation variables for the fr, fl, br and bl
        self.bl_active = 0
        self.fr_active = 0
        self.br_active = 0
        self.fl_active = 0

        # 
        self.little_arm_rate = 0
        self.fr_rate = 0
        self.fl_rate = 0
        self.br_rate = 0
        self.bl_rate = 0
        self.little_arm_reset = 0
        self.fr_reset = 0
        self.fl_reset = 0
        self.br_reset = 0
        self.bl_reset = 0
        # lista de elementos de filtrado
        # self.exp_term_angular_rate = 0
        # self.exp_term_lin_prod = 0   
        # self.last_lin_prod = 0
        # self.last_angular_rate = 0        

        self.base_des = 0       

        self.joySub = rospy.Subscriber("joy", Joy, self.joyCb)
        
    def joyCb(self, data):
		
        # data catch. Los métodos de callback deberían ser cortos y 
        # eficientes, además de imposibles de "bloquear"
        # self.reverse = data.buttons[self.buttons_names['reverse']]
        self.angular_rate = data.axes[self.axes_names['h_left_pad']] * 100
        self.linear_rate = data.axes[self.axes_names['v_left_pad']] * 100

        self.base_rate += data.axes[self.axes_names['h_arrow']] * -.1

        self.bl_active = data.buttons[self.buttons_names['left_trigger_button']]
        self.fr_active = data.axes[self.axes_names['right_trigger_pad']] - 1
        self.br_active = data.buttons[self.buttons_names['right_trigger_button']]
        self.fl_active = data.axes[self.axes_names['left_trigger_pad']] - 1 

        self.little_arm_rate = data.axes[self.axes_names['v_right_pad']] * 10.

        self.little_arm_reset = data.buttons[self.buttons_names['right_pad_button']]

        
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
        
        left_des = self.linear_rate + self.angular_rate
        right_des = self.linear_rate - self.angular_rate

        left_des = self.constrain(left_des,-100,100)
        right_des = self.constrain(right_des,-100,100)

        # self.wheelPub.publish(self.wh)
        # self.
        self.leftPub.publish(left_des)
        self.rightPub.publish(right_des)

        print "left: " + str(left_des)
        print "right: " + str(right_des)

    def motorArmUpdate(self):

        self.basePub.publish(self.base_rate)
        print "base: " + str(self.base_rate)

    def motorTractionArmUpdate(self):

        # motor_traction_arm_fr_rate = 0
        # motor_traction_arm_fl_rate = 0
        # motor_traction_arm_br_rate = 0
        # motor_traction_arm_bl_rate = 0

        if self.fr_active < -1:
            self.fr_rate = self.little_arm_rate
        else:
            self.fr_rate = 0
        if self.fl_active < -1:
            self.fl_rate = self.little_arm_rate
        else:
            self.fl_rate = 0
        if self.br_active:
            self.br_rate = self.little_arm_rate
        else:
            self.br_rate = 0
        if self.bl_active:
            self.bl_rate = self.little_arm_rate
        else:
            self.bl_rate = 0


        if self.little_arm_reset == 1:
            if self.fr_active:
                self.fr_rate = 0
                self.frResetPub.publish(1)
            if self.fl_active:
                self.fl_rate = 0
                self.flResetPub.publish(1)
            if self.br_active:
                self.br_rate = 0
                self.brResetPub.publish(1)
            if self.bl_active:
                self.bl_rate = 0
                self.blResetPub.publish(1)

        # self.motor_traction_arm_fr_rate = self.constrain(self.motor_traction_arm_fr_rate, -100, 100);
        # self.motor_traction_arm_fl_rate = self.constrain(self.motor_traction_arm_fl_rate, -100, 100);
        # self.motor_traction_arm_br_rate = self.constrain(self.motor_traction_arm_br_rate, -100, 100);
        # self.motor_traction_arm_bl_rate = self.constrain(self.motor_traction_arm_bl_rate, -100, 100);                                                 

        self.frPub.publish(self.fr_rate * -1)
        self.flPub.publish(self.fl_rate * -1)
        self.brPub.publish(self.br_rate)
        self.blPub.publish(self.bl_rate * -1)

        print "fr: " + str(self.fr_rate)
        print "fl: " + str(self.fl_rate)
        print "br: " + str(self.br_rate)
        print "bl: " + str(self.bl_rate)
        
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
        return value

# Invocación del constructor. Básicamente, si este archivo fuera a ejecutarse como "top level", el
# intérprete le asignaría el "nombre" (protegido) __main__ y ejecutaría las siguientes lineas como
# LO PRIMERO en el archivo (salvo imports y declaraciones)    
if __name__ == '__main__':
    """ main """
    finder_diff_joy = Finder_diff_joy()
    finder_diff_joy.spin()
    

