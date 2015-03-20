#!/usr/bin/env python
# -*- coding: utf8 -*-

"""
necesito de un metodo que haga uso del tiempo de manera que, dado un tiempo inicial, 
limite el cambio de alguna variable, digamos, que partiendo de 0, si envio un pulso
de 100, pero establezco un cambio maximo de 20 pasos por segundo, entonces llegue al
valor de cien en un tiempo de cinco segundos, pero si hubiera dicho cuarenta, que
haya un primer cambio de 20 durante un segundo y posteriormente uno de 10, en medio
segundo (limitando la pendiente). Por ende, es razonable que se trate como un perfil
de velocidad para las salidas a los motores, conjuntando la idea de una clase de 
limitacion para evitar el que se quemen los motores, y la idea de un perfil de
velocidad para evitar que den golpes subitos

ademas de eso, dado que a menos velocidad de movimiento de los motores, mayor es el 
pwm necesario para mantenerlos en una posicion fija, quiza por efectos de inercia, 
creo conveniente que haya algun metodo que establezca un limite mayor a menos 
velocidad, pero probablemente deba ser una modificacion a las clases de control que
se desplieguen en el siste

necestio hacer un nuevo shield para el bbb, de manera que pueda leer los encoders de
manera directa, y las imus, y se de la alimentacion de 3.3V y una regulacion de 5V y
se tengan los capacitores de regulacion, y las salidas a servos sin que les moleste
el estado del sistema a mas bajo nivel

---> problema, y si el programa pnincipal falla? los servos se irian sin posibilidad 
de regresar rapidamente!!! y eso es muy muy malo
problema, no podemos garantizar tiempos para los encoders, igual y no importa, por la
senal de reloj que se tiene controlada desde la plataforma
lo de los servos es importante, pero porque hay colision con las interrupciones? lo
podria atribuir al serial, o a que ros tome control sobre alguna forma de reset???
no creo, el tiempo en pruebas anteriores sugiere que no, pero mover las salidas a 
servos a mayor prioridad
"""

################################################################################
from math import *
################################################################################
from ino_mod import *
################################################################################
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
################################################################################

"""
por ende, se necesita de un valor de maxima velocidad
se necesita de un valor de salida maxima
se necesita de una pendiente de proteccion contra sobrecalentamiento al estilo del antiguo limite
se necesita saber del rate
se necesita saber el ultimo valor de salida
se necesita de una curva de relacion salida maxima contra velocidad leida de los encoders
"""

"""
los valores deberan darse en una relacion de 0 por ciento al cien por ciento, la conversion a centesimos 
para la salida al micro siempre debera dejarse para el final, para dar la publicacion solamente y mantener
consistencia entre varios programas, agnostico de implementacion tambien, el rate en hertz, etc...
"""

"""
esta clase debra emplearse para la limitacion del control de posicion 
"""

class Profile:
    #
    # {'max_output' : 20, 'max_speed' : 10, 'rate' : 20, 'max_output_at_0RPS' : 30, 'max_output_at_1RPS' : 10, 'heal_time_at_0pc' : 20, 'stable_point_pc' : 20})
    #
    def __init__(self, settings = {'max_output' : 20, 'max_speed' : 20, 'rate' : 20, 'heal_time_at_0pc' : 20, 'stable_point_pc' : 20}):
        #
        self.max_output 		= float(settings['max_output'])
        self.max_speed   		= float(settings['max_speed'])
        self.rate               = float(settings['rate'])
        self.heal_time_at_0pc   = float(settings['heal_time_at_0pc'])
        self.stable_point_pc    = float(settings['stable_point_pc'])
        #
        self.output_actual      = 0.
        self.output_des	        = 0.
        self.last_output_actual = 0.
        self.last_output_des    = 0.
        self.output_limit   	= 0.
        #
        self.x_extend           = self.stable_point_pc
        self.y_extend           = -100. / self.heal_time_at_0pc
        self.m_signed           = self.y_extend / self.x_extend
        self.y0_offset          = 100. / self.heal_time_at_0pc
        #
    def gainFcn(self, data):
        #
        # give me the pc gain per second
        #
        return (self.y0_offset + self.m_signed * abs(data))
        #
    def compute(self, data):
        #
        # var update
        #
        self.last_output_actual = self.output_actual
        #
        self.output_des = data
        #
        # try to limi tot prevent motor overheating
        # convert to the proper value to per callback instead of seconds
        # also, limit by max output
        #
        self.gain_val = self.gainFcn(self.output_actual) / self.rate
        #
        self.output_limit = constrain(self.output_limit + self.gain_val, 0, self.max_output)
        #
        if abs(self.output_des) > abs(self.output_limit):
            self.output_actual = self.output_limit * sign(self.output_des)
        else:
            self.output_actual = self.output_des
        #
        # try to limit output change due to specified maximal speed
        #
        self.output_change = self.output_actual - self.last_output_actual
        #
        if (abs(self.output_change) > self.max_speed / self.rate):
        	self.output_actual = constrain(self.output_actual, self.last_output_actual - self.max_speed / self.rate, self.last_output_actual + self.max_speed / self.rate)      
        #
        return self.output_actual
        #
    def resetting(self, settings):
        #
        self.max_output 		= float(settings['max_output'])
        self.max_speed   		= float(settings['max_speed'])
        self.rate               = float(settings['rate'])
        self.heal_time_at_0pc   = float(settings['heal_time_at_0pc'])
        self.stable_point_pc    = float(settings['stable_point_pc'])
        #
        self.x_extend           = self.stable_point_pc
        self.y_extend           = -100. / self.heal_time_at_0pc
        self.m_signed           = self.y_extend / self.x_extend
        self.y0_offset          = 100. / self.heal_time_at_0pc
        #