#!/usr/bin/env python
# -*- coding: utf8 -*-
#
################################################################################
from math import *
################################################################################
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
################################################################################
from volti.cfg import VOLConfig
################################################################################
from ino_mod import *
from kfilter import *
from pid_pos import *
from pid_vel import *
from encoder import *
from profile import *
################################################################################
from numpy import *
from numpy.linalg import inv 
################################################################################
from volti.msg import float32_3
from volti.msg import float32_12
################################################################################
#
class Double_motor:
    #
    def __init__(self, node_name_default = 'double_motor_adjust'):
        #
        self.GRLinit(node_name_default)
        self.SRVinit()
        #
        # creando los objetos PID
        #self.pid_pos_m1          = PID_pos(self.pos_settings)
        self.pid_vel_m1          = PID_vel(self.vel_settings)
        #self.pid_pos_m2          = PID_pos(self.pos_settings)
        self.pid_vel_m2          = PID_vel(self.vel_settings)
        #
        self.pid_vel_vel         = PID_vel(self.vel_settings)
        #self.pid_pos_ang         = PID_pos(self.pos_settings)
        #
        self.integral_ang        = 0
        self.integral_vel        = 0
        #
        # filtro de entradas del encoder
        self.kf_settings         = {'Q' : 10, 'R' : 10, 'P0' : 10, 'rate' : self.rate}
        #
        # filtros kalman de los encoders
        self.filter_e1           = Kfilter(self.kf_settings)
        self.filter_e2           = Kfilter(self.kf_settings)
        self.filter_plate        = Kfilter(self.kf_settings)
        self.filter_pendu        = Kfilter(self.kf_settings)
        #
        # objeto de lectura del encoder
        self.enc_settings = {'offset' : -1}
        #
        self.enc_1 = Encoder(self.enc_settings)
        self.enc_2 = Encoder(self.enc_settings)
        #
        # Objeto de limitacion de la salida a los motores
        self.profile_settings = {'max_output' : 20, 'max_speed' : 10000, 'rate' : self.rate, 'heal_time_at_0pc' : 20, 'stable_point_pc' : 20}
        #
        self.profile_m1 = Profile(self.profile_settings)
        self.profile_m2 = Profile(self.profile_settings)
        #
        self.des_m1 = 0
        self.value_m1 = 0
        self.angle_m1 = 0
        self.speed_m1 = 0
        self.accel_m1 = 0
        #
        self.des_m2 = 0
        self.value_m2 = 0
        self.angle_m2 = 0
        self.speed_m2 = 0
        self.accel_m2 = 0
        #
        self.vel_del_des = 0
        self.ang_lat_des = 0
        #
        self.rollPlate  = 0
        self.pitchPlate = 0
        self.rollPendu  = 0
        self.pitchPendu = 0
        #
        self.powtog = 0
        self.con_mode = 0
        #
        self.powsub = rospy.Subscriber("con_mode", Int16, self.modecb)
        #
        self.angmultval = 1
        self.velmultval = 1
        #
        self.angmultsub = rospy.Subscriber("ang_mult", Float32, self.angmultcb)
        self.velmultsub = rospy.Subscriber("vel_mult", Float32, self.velmultcb)
        #
        # Asociaciones con publicadores y suscriptores
        self.m1    = rospy.Publisher( "m1",    Int16)              # salida al motor 1
        self.e1    = rospy.Subscriber("e1",    Int16, self.e1cb)  # entrada del encoder 1
        self.e1ang = rospy.Publisher("e1_ang", Float32)     #
        self.e1vel = rospy.Publisher("e1_vel", Float32)     #
        #
        self.m2    = rospy.Publisher( "m2",    Int16)              # salida al motor 1
        self.e2    = rospy.Subscriber("e2",    Int16, self.e2cb)  # entrada del encoder 1
        self.e2ang = rospy.Publisher("e2_ang", Float32)     #
        self.e2vel = rospy.Publisher("e2_vel", Float32)     #
        #
        #self.vdes = rospy.Subscriber("vdes", Float32, self.vdescb)  # velocidad deseada adelante atras
        #
        #
        self.srv = Server(VOLConfig, self.SRVcallback)
        #self.srvenc = Server(ENCConfig, self.ENCcallback)
        #
        self.normalizedkp0rps = 1
        self.normalizedkp1rps = 1
        self.normalizedkpvel  = 1
        self.normalizedkmvel  = 1
        #
        self.rollPenduPub = rospy.Publisher( "rpendu", Float32)
        self.rollPlatePub = rospy.Publisher( "rplate", Float32)
        self.anglatdifPub = rospy.Publisher( "angdif", Float32)
        self.minierrorPub = rospy.Publisher( "minerr", Float32)
        self.outpidangPub = rospy.Publisher( "outang", Float32)
        #
        self.veldeldessub = rospy.Subscriber("vel_delante_des", Float32, self.veldeldescb)
        self.anglatdessub = rospy.Subscriber("ang_lateral_des", Float32, self.anglatdescb)
        #
        self.subImuPlate = rospy.Subscriber('imu_plate_3', float32_3, self.imuplatecb)
        self.subImuPendu = rospy.Subscriber('imu_pendu_3', float32_3, self.imupenducb)
        #
    def angmultcb(self, data):
        #
        self.angmultval = data.data
        #
    def velmultcb(self, data):
        #
        self.velmultval = data.data
        #
    def modecb(self, data):
        #
        self.con_mode = data.data
        #
    def veldeldescb(self, data):
        #
        self.vel_del_des = data.data
        #
    def anglatdescb(self, data):
        #
        self.ang_lat_des = data.data
        #
    def imuplatecb(self, data):
        #
        self.rollPlate  = data.data[0] - 0.06
        self.pitchPlate = data.data[1]
        #
    def imupenducb(self, data):
        #
        #self.rollPendu  = self.angleRange(data.data[0] + 3.1416)
        if (data.data[0] < 0):
            self.rollPendu = data.data[0] + 3.1416
        if (data.data[0] > 0):
            self.rollPendu = data.data[0] - 3.1416
        self.rollPendu = self.rollPendu + 0.05
        self.pitchPendu = data.data[1]
        #
    def GRLinit(self, node_name_default):
        #
        rospy.init_node(node_name_default)
        self.nodename = rospy.get_name()
        rospy.loginfo("Started node %s", self.nodename)
        #
        self.rate = float(rospy.get_param("rate", 10))
        self.times = 0
        #
    def SRVinit(self):
        #
        # pid_out da el valor de salida al motor de -100 a 100
        # pid_ang da el valor del angulo del encoder en su marco de referencia (lafrom std_msgs.msg import Bool IMU da el angulo de la esfera, en general, dado
        # que cada motor puede dar una cantidad potencialmente infinita de vueltas, no es posible hacer una correspondencia uno a uno
        # con el angulo real del motor. Los encoders se consideran digitales de 0 a 1023 en una vuelta completa
        # pid_vel da el valor de la velocidad del angulo del encoder en su marco de referencia, se calcula en base al valor anterior
        #
        self.pos_settings = {'kp0rps' : 0, 'kp1rps' : 0, 'ki' : 0, 'kd' : 0, 'ka' : 0, 'ks' : 0, 'umbral' : 0, 'umbral_int' : 0, 'umbral_oof' : 0, 'range' : 0, 'rate' : self.rate}
        self.vel_settings = {'kp' : 0, 'ki' : 0, 'kd' : 0, 'km' : 0, 'ka' : 0, 'ks' : 0, 'umbral' : 0, 'ki_dec' : 0, 'range' : 0, 'rate' : self.rate}
        #
    def SRVcallback(self, config, level):
        #
        rospy.loginfo("reconfiguring")
        #
        #values catched from mouse event from rqt_gui
        self.normalizedkp0rps = float(config['kp0rps_pos'])      
        self.normalizedkp1rps = float(config['kp1rps_pos']) 
        self.normalizedkpvel  = float(config['kp_vel'])
        self.normalizedkmvel  = float(config['km_vel']) 
        #
        self.pos_settings['kp0rps'] = float(config['kp0rps_pos'])      
        self.pos_settings['kp1rps'] = float(config['kp1rps_pos'])      
        self.pos_settings['ki']     = float(config['ki_pos'])           
        self.pos_settings['kd']     = float(config['kd_pos'])
        self.pos_settings['ka']     = float(config['ka_pos'])           
        self.pos_settings['ks']     = float(config['ks_pos'])              
        # variables de limitacion adicionales
        self.pos_settings['umbral']     = float(config['umbral_pos'])
        self.pos_settings['umbral_int'] = float(config['umbral_int'])
        self.pos_settings['umbral_oof'] = float(config['umbral_oof'])
        self.pos_settings['range']      = float(config['range_pos'])
        # parametros de posicion, hacer parametros posteriormente, usar dynamic
        self.vel_settings['kp']     = float(config['kp_vel'])          
        self.vel_settings['ki']     = float(config['ki_vel'])          
        self.vel_settings['kd']     = float(config['kd_vel'])          
        self.vel_settings['km']     = float(config['km_vel']) 
        self.vel_settings['ka']     = float(config['ka_vel'])          
        self.vel_settings['ks']     = float(config['ks_vel'])             
        # variables de limitacion adicionales
        self.vel_settings['umbral'] = float(config['umbral_vel'])
        self.vel_settings['range']  = float(config['range_vel'])
        #
        #self.pid_pos_ang.resetting(self.pos_settings)
        self.pid_vel_vel.resetting(self.vel_settings)
        #
        self.kf_settings['P0']   = float(config['P0'])
        self.kf_settings['Q']    = float(config['Q'])
        self.kf_settings['R']    = float(config['R'])
        self.kf_settings['rate'] = float(config['rate'])
        #
        self.filter_e1.resetting(self.kf_settings)
        self.filter_e1.resetting(self.kf_settings)
        self.filter_plate.resetting(self.kf_settings)
        self.filter_pendu.resetting(self.kf_settings)
        #
        return config
        #
    def e1cb(self, data):
        #
        self.value_m1 = data.data
        self.proc_m1  = self.enc_1.compute(self.value_m1)
        self.X_m1     = self.filter_e1.compute(self.proc_m1)
        #
        self.angle_m1 = self.X_m1[0, 0]
        self.speed_m1 = self.X_m1[1, 0]
        #self.accel_m1 = self.X_m1[2, 0]
        #
        self.e1ang.publish(self.angle_m1)
        self.e1vel.publish(self.speed_m1)
        #
    def e2cb(self, data):
        #
        self.value_m2 = data.data
        self.proc_m2  = self.enc_2.compute(self.value_m2)
        self.X_m2     = self.filter_e2.compute(self.proc_m2)
        #
        self.angle_m2 = self.X_m2[0, 0]
        self.speed_m2 = self.X_m2[1, 0]
        #self.accel_m2 = self.X_m2[2, 0]
        #
        self.e2ang.publish(self.angle_m2)
        self.e2vel.publish(self.speed_m2)
        #
    def getKp(self, speed):
        #
        return constrain(self.pos_settings['kp0rps'] - (self.pos_settings['kp0rps'] - self.pos_settings['kp1rps']) * abs(speed), 0, 10000)
        #
    def controller(self):
        #
        #
        # Una pmejor ley de control necesita de los siguientes parametros, ignrando por ahora la parte integral
        # o anadiendola de forma posterior, una mejor aproximacion podria ser incluyendo o excluyendo terminos de:
        #
        # K1 * vel + K2 * ang + K3 * accel + K4 * sen(ang)
        #
        # la orientacion de los motores en el estado actual es tal que si m2 recibe una salida positiva, y m1 su 
        # contrario, el angulo tendera a incrementarse a valores positivos
        #
        # Necesitamos las varaibles que puede dar la IMU
        #
        # como angulo:
        #
        self.ang_plate = self.rollPlate
        self.ang_pendu = self.rollPendu
        #
        # pero quiza necesitamos de filtrar las variables para poder encontrar las velocidades, y en eso no 
        # estamos el retraso que pueda darse en el procesamiento
        #
        self.X_plate   = self.filter_plate.compute(self.ang_plate)
        self.X_pendu   = self.filter_pendu.compute(self.ang_pendu)
        #
        #self.ang_plate = self.X_plate[0, 0]
        #self.ang_pendu = self.X_pendu[0, 0]
        #
        self.vel_plate = self.X_plate[1, 0]
        self.vel_pendu = self.X_pendu[1, 0]
        #
        # no se las aceleraciones, pareceria ser algo inestable y problematico, pero podria estar bien
        #
        self.ace_plate = self.X_plate[2, 0]
        self.ace_pendu = self.X_pendu[2, 0]
        #
        # se necesita conocer la velocidad de los motores, para poder dar una kp variable
        #
        self.avg_vel_m1_m2 = (self.speed_m1 + self.speed_m2) / 2;
        #
        self.rollPenduPub.publish(self.ang_pendu)
        self.rollPlatePub.publish(self.ang_plate)
        #
        # como sea, es el angulo del pendulo el que tiene a subir con m2 positiva, y lo que queremos es estabilixar...
        # que? podria parecer ser el pendulo, pero no estoy tan seguro de que sea la mejor opcion, 
        #
        # no podemos o querriamos operar sobre la diferencia, porque podria salir mal, en el caso de que los angulos
        # coincidan la accion de control seria inutil
        # no podemos operar directamente sobre la placa, pero si podemos operar directamente sobre el pendulo...
        # pero el pendulo y la placa actuan de formas opuestas, aumentar el angulo del pendulo es lo mismo que disminuir
        # el angulo de la placa, bajo algun retraso introducido por el sistema, etc.
        # podria recomendar hacer simulacions mejores, pero hemos visto que no son necesariamente utiles para el 
        # funcionamiento del sistema
        #
        # ok, propongamos entonces suponer que tratamos con un sistema con alta inercia o alta friccion, prob. mas inercia,
        # controlando la placa, entones, las ecuaciones de control posibles podrian ser simplemente:
        #
        # dar una salia positiva a m2 incrementa el angulo del endulo y por ende disminuye el de la placa, ok, aunque la tendencia
        # del angulo del pendulo sea siempre a buscar el equilibiro, ok ok
        #
        # PWM_m1 = -K1 * vel_plate - K2 * ang_plate - K3 * ace_plate - K4 * sen(ang) - K5 * sum(ang_plate)
        #
        # la parte importante adicional podria ser la adicion del termino de angulo, para bajos angulos seria muy
        # similar a un termino de angulo adicional... no se si funcione correctamente
        #
        # los problemas actuales con el control son:
        #
        # que los motores necesitan de un valor de pwm minimo para accionarse por la necesidad de romper la inercia del sistema
        # de aproximadamente el 4% del pwm total, esto esta documentado tambien el el manual del talon
        # -> establecer un floor para las salidas de los motores, aunque podrian desgastarse los carbones mucho mas rapido
        #
        # la parte integral introduce innecesraias complicaciones cerca del cero, porque su descarga tiene a que el sistema oscile por un
        # rato
        # -> eliminar la parte integral cerca de un entorno del cero, pero mover al sistema con mas energia con el resto de los
        # parametros del controlador
        #
        # el procesamiento de las imus es lento, intentare ver que pasa si se incrementa la velocidad de sampling, como ya se hace, y se
        # reintroduce lo de incrementar ligeramente la contribucion de los giroscopios
        # -> incrementar el valor de las lecturas de los giroscopios un 5%, pero pasar las lecturas por un filtro kalman tambien
        #
        # todavia parece haver un problema de desequilibrio del sistema, tal vez pueda ajustarse mejor, tal vez, tal vez
        #
        # ok, pues tendria que hacerso todo aqui mismo, pero deberia preservarse la salida a traves del perfil por proteccion al sistema
        # pero nunca pude implementarla de tal forma que fuera muy asimetrica, no se, no era tan facil...
        # 
        # entonces, podria ser algo como
        #
        if abs(self.ang_plate) < self.pos_settings['umbral']:
            self.ang_plate_tmp = 0
        else:
            self.ang_plate_tmp = self.ang_plate
        # 
        self.salida_m1_ang = self.getKp(self.avg_vel_m1_m2) * self.ang_plate_tmp + self.pos_settings['kd'] * self.vel_plate + self.pos_settings['ka'] * self.ace_plate - self.pos_settings['ks'] * sin(self.ang_plate) 
        #
        self.integral_ang  = constrain(self.integral_ang + self.ang_plate / self.rate, -1, 1)
        if abs(self.ang_plate) < self.pos_settings['umbral_int']:
             self.integral_ang = 0
        #
        if abs(self.ang_plate) < self.pos_settings['umbral_oof']:
            self.salida_m1_ang = sign(self.salida_m1_ang) * 5 * 1 / (1 + 19 * self.avg_vel_m1_m2)
        else:
            if abs(self.salida_m1_ang) < 4 and abs(self.salida_m1_ang) > 0.5:
                self.salida_m1_ang = sign(self.salida_m1_ang) * 5 * 1 / (1 + 19 * self.avg_vel_m1_m2)
        #
        # implementando los umbrales
        # umbral      -> cero err
        # umbral_int  -> cero int
        # umbral_oof  -> +-4%
        #
        self.salida_m1_ang = constrain(self.salida_m1_ang, -self.pos_settings['range'], self.pos_settings['range'])
        self.salida_m1_ang = self.salida_m1_ang + self.pos_settings['ki'] * self.integral_ang
        #
        #self.salida_control_vel = self.pid_vel_vel.compute(self.vel_del_des, self.avg_vel_m1_m2, 0)
        #
        self.salida_m1_vel = self.pid_vel_m1.compute(self.vel_del_des + self.ang_plate / 5, self.speed_m1, 0)
        self.salida_m2_vel = self.pid_vel_m2.compute(self.vel_del_des - self.ang_plate / 5, self.speed_m2, 0)
        #
        self.salida_control_vel = constrain(self.salida_control_vel, -20, 20)
        #
        self.out_pos_m1 = self.profile_m1.compute( self.salida_m1_ang + self.salida_m1_vel )
        self.out_pos_m2 = self.profile_m2.compute( -self.salida_m1_ang + self.salida_m2_vel )
        #
        #
        # the control interface is publishing motor pwm values directly
        # though, I would prefer to vanish the integral term in this case, and fix the desired angle and speed, etc.
        #
        if (self.con_mode is 1):
            return
        #
        self.m1.publish(int((self.out_pos_m1) * 100))
        self.m2.publish(int((self.out_pos_m2) * 100))
        #
        #
    def update(self):
        #
        self.times = self.times + 1
        #
        self.controller()
        #
        # the interaction between modifying the values in rqt at the same time that with the joystick could
        # lead to runaway exponential growth, not catched by the rqt applet, be careful
        #
        # a possible solutino would be to avoid setting the paramters, and just pass to the pid reconfigure methods, but
        # i feel that would be expensive
        #
        # you dont know if that is necessary...
        #
        # because the methods are not updated outside of a dyn reconf event!!!
        # a call here would be expensive, but useful regardless...
        #        
        # do every second? its not another node, after all, overhead should be smalll
        #
        #
        #
        # self.roll_des_val
        #
        #self.roll_diff_act_val  = self.rollPlate - self.rollPendu - 0.126
        #self.roll_diff_des_val  = self.roll_des_val
        #
        #rospy.loginfo("rolldiffact: " + str(self.roll_diff_act_val) + " rolldiffdes: " + str(self.roll_diff_des_val))
        #
        #self.out_pos_m1 = -self.pid_pos_m1.compute(self.roll_diff_des_val, self.roll_diff_act_val, 0)
        #self.limited_out_m1 = constrain(self.out_pos_m1 + 100, -500, 500)#self.limit_m1.compute(self.final_out_m1)
        #self.m1.publish(self.limited_out_m1)
        #self.out_pos_m2 = self.pid_pos_m2.compute(self.roll_diff_des_val, self.roll_diff_act_val, 0)
        #self.limited_out_m2 = constrain(self.out_pos_m2 + 100, -500, 500)#self.limit_m1.compute(self.final_out_m1)
        #self.m2.publish(self.limited_out_m2)
        #
        #
        #
        #
        #
    def spin(self):
        #
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
            #
if __name__ == '__main__':
    #
    """ main """
    double_motor = Double_motor()
    double_motor.spin()
    #