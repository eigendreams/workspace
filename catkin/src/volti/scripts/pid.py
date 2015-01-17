#!/usr/bin/env python
# -*- coding: utf8 -*-
################################################################################
from math import *
################################################################################
from ino_mod import *
################################################################################
#
class PID:
    #
    # NO DEFAULTS !!!
    #
    def __init__(self, settings):
        #
        self.kp = settings['kp']
        self.ki = settings['ki']
        self.kd = settings['kd']
        self.km = settings['km']
        #
        self.umbral = settings['umbral']        # valor por debajo del cual el error podria ser ruido
        self.ki_dec = settings['ki_dec']        # ki de penalizacion
        self.range  = settings['range']         # maxima salida posible, se panalizara primero a la parte integral y luego a la demas
        #
        self.kisum          = 0
        self.error          = 0
        self.last_error     = 0
        self.derror         = 0                         # FILTRAR
        self.pid_out        = 0
        self.times          = 0
        #
        #
    def compute(self, desired, actual, accel):
        #
        self.last_error = self.error
        self.error = desired - actual
        #
        if (accel == None):
            self.derror = self.error - self.last_error
        else:
            self.derror = accel
        #
        if (self.times == 0):
            self.derror = 0
        self.times = self.times + 1
        #
        self.kisum = constrain(self.kisum + self.error * (self.ki if (sign(self.kisum) == sign(self.error)) else self.ki_dec), -self.range, self.range)
        if (abs(self.error) < self.umbral):
            self.error = 0
        self.pid_out = self.kp * self.error - self.kd * self.derror + self.km * desired
        #
        self.kisum = sign(self.kisum) * min(abs(self.kisum - sign(self.kisum) * max(abs(self.kisum + self.pid_out) - self.umbral, 0)), abs(self.kisum))
        self.pid_out = self.pid_out + self.kisum
        #
        return self.pid_out
        #
        #
    def reset(self):
        #
        self.pid_out = 0
        self.kisum = 0
        #
        #
    def resetting(self, settings):
        #
        self.kp = settings['kp']
        self.ki = settings['ki']
        self.kd = settings['kd']
        self.km = settings['km']
        #
        self.umbral = settings['umbral']        # valor por debajo del cual el error podria ser ruido
        self.ki_dec = settings['ki_dec']        # ki de penalizacion
        self.range  = settings['range']         # maxima salida posible, se panalizara primero a la parte integral y luego a la demas   