#!/usr/bin/env python
# -*- coding: utf8 -*-
################################################################################
from math import *
################################################################################
from ino_mod import *
################################################################################
#
"""
Tratar de pasar de un sistema de encoders digitales de 0 a 1023 a un valor continuo en radianes que
podria no tener limites, la primera lectura, por la disposicion mecanica, se toma como un cero de angulo,
pero lo que se quiera controlar del motor sera velocidad, y de esta forma no deberia ser problematico con la ayuda
de los sensores externos como la imu
Esta clase no debe hacer ninguna forma de filtrado o derivacion, de ello se encargaran clases posteriores, solo debe
devolver el valor del angulo sin limite del encoder en formato binario? en angulo?... Yo recomendaria en angulo para 
reducir las modificaiones posibles a estapas superiores
"""
#
class Encoder:
    #
    def __init__(self, settings = {'offset' : -1}):
        #
        self.offset = settings['offset']
        self.times  = 0
        self.angle  = 0
        self.laps   = 0
        #
    def compute(self, measure):
        #
        if (measure == -1):
            # tendria que adivinar que el encoder se sigue moviendo, probablemente a una velocidad constante, entonces, para UN SOLO PUNTO DE ERROR
            #
            self.output_angle = self.output_angle + self.corrected_change
            #
            # no hacemos alguna forma de verificacion de que el cambio, en el proximo ciclo, sea el correcto, para que en dos puntos fallemos, deberiamos
            # movernos a un cambio de 512 pasos entre esos dos puntos, suponiendo que solo falle el primer punto, por ende, fallariamos a una velocidad de
            # de movimiento del motor, a un rate de 20 hz, de 512 pasos en un decimo de segundo, lo que serian 5 RPS (creo que es condierable)
            # si los errores ocntinuan estamos en un problema, alive debera matar los motores en el tiempo que resta
            #
            # esto es para que no se pierda el filtro de la velocidad para errores transientes, y nada mas para eso? 
            #
            # y que tal que los erroes sigan? no me parece que esto tenga tanto sentido, en este caso lo mejor sea ignorar el dato erroneo
            # pero estimar o no estimar el angulo destino?
            # yo propondria que no, pero como hay un controlador de velocidad, pero toma de las imus, pero no se...
            # 
            # depende de las pruebas, las pruebas muestran que los errores son en ocasiones solamente puntuales, pero uno de los encoders parecia ser algo
            # renuente... y daba valores de error, por periodos mas largos, porque??? hacemos verificacion y eso pero no parecia ser suficiente? parecian ser falsos???
            # deberiamos depender de las imus solamente???
            #
            # la naturaleza de los errores parecia ser transiente y puntual, con eso en mente, esta aproximacion esta bien
            #
            # ok
            #
            return self.output_angle
        #
        if (self.times == 0):
            self.internal_angle_last    = map(measure, 0., 1023., 0., 2. * pi)
            self.internal_angle         = map(measure, 0., 1023., 0., 2. * pi)
            if (self.offset < 0):
                self.internal_offset    = map(measure, 0., 1023., 0., 2. * pi)
            else:
                self.internal_offset    = map(self.offset, 0., 1023., 0., 2. * pi)
        #
        self.internal_angle_last        = self.internal_angle
        self.internal_angle             = map(measure, 0., 1023., 0., 2. * pi)
        #
        self.change = self.internal_angle - self.internal_angle_last
        if (abs(self.change) > pi): # es un cambio muy drastico, asumiremos que hubo un salto,  max RPM de 300
            if (self.internal_angle_last > self.internal_angle): # ej saltar de 900 a 100 -> 100 -800 -> change de 800 
                self.laps = self.laps + 1
            if (self.internal_angle_last <= self.internal_angle): # ej saltar de 100 a 900 -> 900 - 100 -> change de 800
                self.laps = self.laps - 1
        #
        self.output_angle_last          = self,output_angle
        self.output_angle               = 2 * pi * self.laps + self.internal_angle - self.internal_offset
        #
        self.corrected_change = self.output_angle - self.output_angle_last
        #
        self.times                      = self.times + 1
        #
        return self.output_angle
        #
    def resetting(self, settings):
        #
        self.offset = settings['offset']