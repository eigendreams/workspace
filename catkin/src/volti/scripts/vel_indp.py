#!/usr/bin/env python
# -*- coding: utf8 -*-
#
#
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
#from volti.cfg import PIDConfig
#from volti.cfg import ENCConfig
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
        self.pid_pos_m1          = PID_pos(self.pos_settings)
        self.pid_vel_m1          = PID_vel(self.vel_settings)
        self.pid_pos_m2          = PID_pos(self.pos_settings)
        self.pid_vel_m2          = PID_vel(self.vel_settings)
        #
        self.pid_vel_vel         = PID_vel(self.vel_settings)
        self.pid_pos_ang         = PID_pos(self.pos_settings)
        #
        # filtro de entradas del encoder
        self.kf_settings         = {'Q' : 10, 'R' : 10, 'P0' : 10, 'rate' : self.rate}
        #
        # filtros kalman de los encoders
        self.filter_e1           = Kfilter(self.kf_settings)
        self.filter_e2           = Kfilter(self.kf_settings)
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
        self.angle_lock_m1 = 0
        #
        self.des_m2 = 0
        self.value_m2 = 0
        self.angle_m2 = 0
        self.speed_m2 = 0
        self.accel_m2 = 0
        #
        self.angle_lock_m2 = 0
        #
        self.vel_del_des = 0
        self.ang_lat_des = 0
        #
        self.rollPlate  = 0
        self.pitchPlate = 0
        self.rollPendu  = 0
        self.pitchPendu = 0
        #
        self.minimal_error = 0
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
        """
        self.pos_settings['kp0rps'] = float(config['kp0rps_pos'])      
        self.pos_settings['kp1rps'] = float(config['kp1rps_pos'])      
        self.pos_settings['ki']     = float(config['ki_pos'])           
        self.pos_settings['kd']     = float(config['kd_pos'])              
        # variables de limitacion adicionales
        self.pos_settings['umbral'] = float(config['umbral_pos'])
        self.pos_settings['ki_dec'] = float(config['ki_dec_pos'])
        self.pos_settings['range']  = float(config['range_pos'])
        # parametros de posicion, hacer parametros posteriormente, usar dynamic
        self.vel_settings['kp']     = float(config['kp_vel'])          
        self.vel_settings['ki']     = float(config['ki_vel'])          
        self.vel_settings['kd']     = float(config['kd_vel'])          
        self.vel_settings['km']     = float(config['km_vel'])         
        # variables de limitacion adicionales
        self.vel_settings['umbral'] = float(config['umbral_vel'])
        self.vel_settings['ki_dec'] = float(config['ki_dec_vel'])
        self.vel_settings['range']  = float(config['range_vel'])
        """
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
        self.minimal_error = 1
        #
        self.ang_lat_des = data.data
        #
    def angleRange(self, data):
        #
        if (data > 0):
            return data - 3.1416
        if (data < 0):
            return data + 3.1416
        return data
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
        self.pos_settings = {'kp0rps' : 0, 'kp1rps' : 0, 'ki' : 0, 'kd' : 0, 'umbral' : 0, 'ki_dec' : 0, 'range' : 0, 'rate' : self.rate}
        self.vel_settings = {'kp' : 0,         'ki' : 0, 'kd' : 0, 'km' : 0, 'umbral' : 0, 'ki_dec' : 0, 'range' : 0, 'rate' : self.rate}
        # parametros de posicion, hacer parametros posteriormente, usar dynamic
        self.pos_settings['kp0rps'] = float(rospy.get_param('~kp0rps_pos',  '0'))      
        self.pos_settings['kp1rps'] = float(rospy.get_param('~kp1rps_pos',  '0'))      
        self.pos_settings['ki']     = float(rospy.get_param('~ki_pos',  '0'))           
        self.pos_settings['kd']     = float(rospy.get_param('~kd_pos',  '0'))                    
        # variables de limitacion adicionales
        self.pos_settings['umbral'] = float(rospy.get_param('~umbral_pos', '0'))  
        self.pos_settings['ki_dec'] = float(rospy.get_param('~ki_dec',     '0'))  
        self.pos_settings['range']  = float(rospy.get_param('~range_pos',  '10'))
        # parametros de posicion, hacer parametros posteriormente, usar dynamic
        self.vel_settings['kp']     = float(rospy.get_param('~kp_vel',  '0'))           
        self.vel_settings['ki']     = float(rospy.get_param('~ki_vel',  '0'))           
        self.vel_settings['kd']     = float(rospy.get_param('~kd_vel',  '0'))           
        self.vel_settings['km']     = float(rospy.get_param('~km_vel',  '0'))           
        # variables de limitacion adicionales
        self.vel_settings['umbral'] = float(rospy.get_param('~umbral_vel', '0'))  
        self.vel_settings['ki_dec'] = float(rospy.get_param('~ki_vel',     '0'))  
        self.vel_settings['range']  = float(rospy.get_param('~range_vel',  '10'))
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
        # variables de limitacion adicionales
        self.pos_settings['umbral'] = float(config['umbral_pos'])
        self.pos_settings['ki_dec'] = float(config['ki_dec_pos'])
        self.pos_settings['range']  = float(config['range_pos'])
        # parametros de posicion, hacer parametros posteriormente, usar dynamic
        self.vel_settings['kp']     = float(config['kp_vel'])          
        self.vel_settings['ki']     = float(config['ki_vel'])          
        self.vel_settings['kd']     = float(config['kd_vel'])          
        self.vel_settings['km']     = float(config['km_vel'])         
        # variables de limitacion adicionales
        self.vel_settings['umbral'] = float(config['umbral_vel'])
        self.vel_settings['ki_dec'] = float(config['ki_dec_vel'])
        self.vel_settings['range']  = float(config['range_vel'])
        #
        #self.pid_pos_m1.resetting(self.pos_settings)
        #self.pid_vel_m1.resetting(self.vel_settings)
        #
        #self.pid_pos_m2.resetting(self.pos_settings)
        #self.pid_vel_m2.resetting(self.vel_settings)
        #
        self.pid_pos_ang.resetting(self.pos_settings)
        self.pid_vel_vel.resetting(self.vel_settings)
        self.pid_vel_m1.resetting(self.vel_settings)
        self.pid_vel_m2.resetting(self.vel_settings)
        #
        #
        #
        self.kf_settings['P0'] = float(config['P0'])
        self.kf_settings['Q'] = float(config['Q'])
        self.kf_settings['R'] = float(config['R'])
        self.kf_settings['rate'] = float(config['rate'])
        #
        self.filter_e1.resetting(self.kf_settings)
        self.filter_e1.resetting(self.kf_settings)
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
    def controller(self):
        #
        #
        # Quiero controlar el angulo de roll del pendulo, pero verificar contra el angulo de roll de plate para evitar colisiones
        # este control es solamente de posicion, la parte del angulo de pitch se deriva de un controlador de velocidad, que se 
        # superpondra linealmente a las salidas de este controlador, obviamente, la parte del control de velocidad no podria probarse
        # con el pendulo en la base de pruebas
        #
        # la unica forma de sensar la velocidad de movimiento del pendulo seria a traves de los encoders, PERO como tambien se mueve
        # en direccion del roll, no podemos simplemente asumir que es los encoders y ya, pareces ser que habria que considerar la
        # contribucion del angulo de pitc. En general, las ecuaciones son:
        #
        # movimiento hacia adelante = (e1 + e2) / 2 (PITCH_RELATIVE)
        # movimiento hacia los lados = (e1 - e2) / 2 (ROLL_RELATIVE)
        #
        # entonces, para saber el grado de avance hacia adelante, si tenemos la lectura de los encoders pero deseamos quitar el efecto del roll:
        #
        # datos: e1, e2, rolls, pitchs
        # 
        self.velocidad_adelante         = (self.speed_m1 + self.speed_m2) / 2   # no estamos haciendo correcciones, ests es la unica forma de controlar la
        #self.ang_lat_pend               = self.rollPendu # este es el valor que queremos controlar
        #self.ang_lat_diff               = self.rollPlate - self.rollPendu # pero este valor no debe salir de rango de entre -0.5 a 0.5, aprox
        #
        #
        #self.control_var                = self.rollPlate
        #
        #
        #rospy.loginfo("rollPendu: " + str(self.rollPendu) + " rollPlate: " + str(self.rollPlate) + " angdes: " + str(self.ang_lat_des))
        #
        #
        # el control de angulo del pendulo en roll, no podria ser a traves de los encoders... por el juego de las bandas, en primer lugar... por ende este solo
        # podria ser un controlador de posicion, donde
        #
        # speed_m1_setpoint = velocidad_adelante_deseada + velocidad_lateral_deseada
        # speed_m2_setpoint = velocidad_Adelante_deseada - velocidad_lateral_deseada
        #
        # queremos hacer dos cosas, controlar la velocidad_adelante, con una posible contribucion de un angulo de la imu en pitch que no podria probarse
        # en una base de pruebas... then
        #
        # el control de la velocidad deberia hacerse por motor, y con referencia de las velocidades de las imus
        # el control de posicion deberia hacerse global (un unico controlador), y con referencia de los angulos de las imus y un factor integral adicionas
        #
        #
        #
        # pero debemos encontrar alguna forma de mitigar el bamboleo, se me ocurre que tengamos alguno valor de umbral, para el control del angulo, en velocidad
        # no hay un problema tan grande, basicamente, como los motores parece capaces de soportarse a si mismos, por la amplia reduccion, basta con apagar los motores
        # cuando se alcance el objetivo de control, y reiniciar cuando se salga de un umbral de digamos 5 grados, en radianes son 0.1 radianes TOP
        #
        #
        #
        # aunque el controlador sea global, depende de parametros de cada uno de los motores, por ende alguna implementacion futura deberia considerarlos por separado
        # 
        #
        #
        #
        #
        #self.salida_control_angulo  = self.pid_pos_ang.compute(self.ang_lat_des, self.control_var) 
        #
        #
        #if (self.rollPlate > 0):
        #    if (self.ang_lat_des < 0):
        #        self.salida_control_angulo = self.salida_control_angulo * sin(self.rollPlate) / 0.4
        #if (self.rollPlate < 0):
        #    if (self.ang_lat_des > 0):
        #        self.salida_control_angulo = self.salida_control_angulo * sin(self.rollPlate) / 0.4
        #self.salida_control_angulo = self.salida_control_angulo * max((self.rollPendu / 0.6), 0.5)
        #
        #
        #
        self.salida_m1              = self.pid_vel_m1.compute(self.vel_del_des, self.speed_m1, 0)
        self.salida_m2              = self.pid_vel_m2.compute(self.vel_del_des, self.speed_m2, 0)
        #
        #self.salida_control_angulo  = self.salida_control_angulo + sign(self.salida_control_angulo) * 0
        #
        #
        #
        #
        #
        #self.salida_control_vel     = self.pid_vel_vel.compute(self.vel_del_des, self.velocidad_adelante, 0)
        #
        #self.outpidangPub.publish(self.salida_control_angulo)
        #
        #rospy.loginfo("pid: " + str(self.salida_control_angulo))
        #
        #self.salida_control_angulo  = self.salida_control_angulo + 0 * sign(self.salida_control_angulo)
        #
        #
        #rospy.loginfo("salida: " + str(self.salida_control_angulo) + " anglatdiff: " + str(self.ang_lat_diff))
        #
        #
        #
        # esto no basta, tenemos que revisar que no salgamos de los limites de plate y en ese caso habria que apagar los motores, en ese rango, por ahora de inmediato
        #
        #if (self.ang_lat_diff > 0.41):
        #    self.salida_control_angulo = constrain(self.salida_control_angulo, -20, 0)
        #if (self.ang_lat_diff < -0.41):
        #    self.salida_control_angulo = constrain(self.salida_control_angulo, 0, 20)
        #
        #
        #rospy.loginfo("salidaaftlatdiff: " + str(self.salida_control_angulo))
        # si el valor del error minimo baja de cierto umbral, tambien habria que apagar los motores y no encenderlos hasta que se salga de ese umbral o se
        # pida una nueva posicion de control del angulo
        #
        #
        #self.actual_error = self.control_var
        #if (abs(self.actual_error) < self.minimal_error):
        #    self.minimal_error = constrain(abs(self.actual_error), 0, 0.4)
        #
        #self.tmp_constrain = 20 - 7 * (1 - map(self.minimal_error, 0, 0.4, 0, 1))
        #
        #
        #rospy.loginfo("salida: " + str(self.salida_control_angulo) + " minerror: " + str(self.minimal_error))
        # aplica un constrain de salida al motor
        #
        #
        #
        #self.rollPenduPub.publish(self.rollPendu)
        #self.rollPlatePub.publish(self.rollPlate)
        #self.anglatdifPub.publish(self.ang_lat_diff)
        #self.minierrorPub.publish(self.minimal_error)
        #self.veladesumPub.publish(self.velocidad_adelante) 
        #
        #
        #
        #self.salida_control_angulo = constrain(self.salida_control_angulo, -self.tmp_constrain, self.tmp_constrain)
        #self.salida_control_vel    = constrain(self.salida_control_vel, -20, 20)
        #
        #
        self.out_pos_m1 = self.profile_m1.compute(self.salida_m1)
        self.out_pos_m2 = self.profile_m2.compute(self.salida_m2)
        #
        #
        # the control interface is publishing motor pwm values directly
        # though, I would prefer to vanish the integral term in this case, and fix the desired angle and speed, etc.
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
