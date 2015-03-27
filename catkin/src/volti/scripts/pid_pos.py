#!/usr/bin/env python
# -*- coding: utf8 -*-
#
################################################################################
from math import *
################################################################################
from ino_mod import *
################################################################################
#
class PID_pos:
    #
    # NO DEFAULTS !!!
    #
    def __init__(self, settings):
        #
        self.kp_at_0rps     = float(settings['kp0rps'])
        self.kp_at_1rps     = float(settings['kp1rps'])
        self.ki             = float(settings['ki'])
        self.kd             = float(settings['kd'])
        #
        self.umbral         = float(settings['umbral'])        # valor por debajo del cual el error podria ser ruido
        self.ki_dec         = float(settings['ki_dec'])        # ki de penalizacion
        self.range          = float(settings['range'])         #float(settings['range'])         # maxima salida posible, se panalizara primero a la parte integral y luego a la demas
        #
        self.rate           = float(settings['rate'])
        #
        self.kisum          = 0.
        self.error          = 0.
        self.last_error     = 0.
        self.derror         = 0.                               # FILTRAR
        self.pid_out        = 0.
        self.times          = 0.
        #
        #
    def compute(self, desired, actual, speed):
        #
        self.last_error = self.error
        self.error = desired - actual
        #
        if (speed == None):
            # multiply by rate to make units consoistent
            # TODO eval to change to
            # self.derror = (self.actual - self.last_actual) * self.rate
            # to keep consistency
            # i recommend to pass the speed value from the encoders??? error prone, noise, from the imus??? slow, not really motor speed
            # leave it like this, but pass the speed if possible from a higer level 
            self.derror = (self.error - self.last_error) * self.rate
        else:
            self.derror = speed
        #
        # to avoid a kick if the derivate value is present and last error is unknown
        #
        if (self.times <= 1):
            self.derror = 0
        self.times = self.times + 1
        #
        self.kisum = constrain(self.kisum + self.error * (self.ki if (sign(self.kisum) == sign(self.error)) else self.ki_dec), -self.range/5, self.range/5)
        #
        if (abs(self.error) < self.umbral):
            self.error = 0
        #
        self.pid_out = self.getKp(self.derror) * self.error - self.kd * self.derror
        #
        self.kisum = sign(self.kisum) * min(abs(self.kisum - sign(self.kisum) * max(abs(self.kisum + self.pid_out) - self.range, 0)), abs(self.kisum))
        self.pid_out = constrain(self.pid_out + self.kisum, -self.range, self.range)
        #
        return self.pid_out
        #
        #
    def getKp(self, speed):
        #
        return (self.kp_at_0rps - (self.kp_at_0rps - self.kp_at_1rps) * abs(speed) / (2 * pi))
        #
        #
    def reset(self):
        #
        self.pid_out = 0.
        self.kisum = 0.
        self.times = 0
        #
        #
    def resetting(self, settings):
        #
        self.kp_at_1rps     = float(settings['kp0rps'])
        self.kp_at_0rps     = float(settings['kp1rps'])
        self.ki             = float(settings['ki'])
        self.kd             = float(settings['kd'])
        #
        self.umbral         = float(settings['umbral'])        # valor por debajo del cual el error podria ser ruido
        self.ki_dec         = float(settings['ki_dec'])        # ki de penalizacion
        self.range          = float(settings['range'])         #float(settings['range'])         # maxima salida posible, se panalizara primero a la parte integral y luego a la demas
        #
        self.rate           = float(settings['rate'])