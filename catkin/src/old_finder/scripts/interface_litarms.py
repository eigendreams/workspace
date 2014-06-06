#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from finder.msg import FourArmsInt16


class Interface_litarms:
    
    def __init__(self, node_name_override = 'interface_litarms'):
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("interface_litarms starting with name %s", self.nodename)
        self.rate = rospy.get_param("interface_litarms_rate", 10)
        
        self.armsData = FourArmsInt16()
        self.armsData.mtfr = 0
        self.armsData.mtfl = 0
        self.armsData.mtbr = 0
        self.armsData.mtbl = 0
        self.armsPub = rospy.Publisher("traction_arms", FourArmsInt16)

        
        # self.armsActualPos = {
        #     arm1 = 0
        # }

        # self.armsActualPos.arm1 = 0
        # self.armsActualPos.arm2 = 0
        # self.armsActualPos.arm3 = 0
        # self.armsActualPos.arm4 = 0

        # self.armsActualPos = [0,0,0,0]
        self.actual_pos_mtfr = 0;
        self.actual_pos_mtfl = 0;
        self.actual_pos_mtbr = 0;
        self.actual_pos_mtbl = 0;

        # self.armsDesiredPos.arm1 = 0
        # self.armsDesiredPos.arm2 = 0
        # self.armsDesiredPos.arm3 = 0
        # self.armsDesiredPos.arm4 = 0

        # self.armsDesiredPos = [0,0,0,0]
        self.desired_pos_mtfr = 0
        self.desired_pos_mtfl = 0
        self.desired_pos_mtbr = 0
        self.desired_pos_mtbl = 0
        
        self.mtafr = rospy.Subscriber("motor_traction_arm_fr", Int16, self.cbmtafr)
        self.mtafl = rospy.Subscriber("motor_traction_arm_fl", Int16, self.cbmtafl)
        self.mtabr = rospy.Subscriber("motor_traction_arm_br", Int16, self.cbmtabr)
        self.mtabl = rospy.Subscriber("motor_traction_arm_bl", Int16, self.cbmtabl)

        self.mtae = rospy.Subscriber("traction_arms_lec", FourArmsInt16, self.cbmtae)

        self.param_velocity = 50
        self.param_ratio = 0.3

        self.param_direction_mtfr = -1
        self.param_direction_mtfl = -1
        self.param_direction_mtbr = 1
        self.param_direction_mtbl = 1
    
    def cbmtafl(self, data):
        # self.armsData.arm2 = data.data * 4 + 500
        # self.armsDesiredPos[0] = data.data
        self.desired_pos_mtfl = data.data
    def cbmtafr(self, data):
        # self.armsData.arm1 = data.data * 4 + 500
        # self.armsDesiredPos[1] = data.data 
        self.desired_pos_mtfr = data.data
    def cbmtabr(self, data):
        # self.armsData.arm3 = data.data * 4 + 500
        # self.armsDesiredPos[2] = data.data 
        self.desired_pos_mtbr = data.data
    def cbmtabl(self, data):
        # self.armsData.arm4 = data.data * 4 + 500
        # self.armsDesiredPos[3] = data.data 
        self.desired_pos_mtbl = data.data

    def cbmtae(self, data):
        # self.armsActualPos[0] += data.arm1*self.param_ratio*self.param_direction_mtfr
        # self.armsActualPos[1] += data.arm2*self.param_ratio*self.param_direction_mtfl
        # self.armsActualPos[2] += data.arm3*self.param_ratio*self.param_direction_mtbr
        # self.armsActualPos[3] += data.arm4*self.param_ratio*self.param_direction_mtbl
        self.actual_pos_mtfr += data.mtfr*self.param_ratio*self.param_direction_mtfr
        self.actual_pos_mtfl += data.mtfl*self.param_ratio*self.param_direction_mtfl
        self.actual_pos_mtbr += data.mtbr*self.param_ratio*self.param_direction_mtbr
        self.actual_pos_mtbl += data.mtbl*self.param_ratio*self.param_direction_mtbl
        
    def update(self):
        self.armsPub.publish(self.armsData)

    def updateMtafr(self):
        direction = 0
        diferential = self.desired_pos_mtfr - self.actual_pos_mtfr

        if(diferential<0):
            direction = -1
        elif(diferential>0):
            direction = 1
        else:
            direction = 0

        if(abs(diferential)>self.param_velocity):
            diferential=self.param_velocity*direction

        self.armsData.mtfr = diferential*self.param_direction_mtfr

        print "Direccion FR: " + str(diferential*self.param_direction_mtfr)
        print "Posicion actual FR: " + str(self.actual_pos_mtfr)
        print "Posicion deseada FR: " + str(self.desired_pos_mtfr)

    def updateMtafl(self):
        direction = 0
        diferential = self.desired_pos_mtfl - self.actual_pos_mtfl

        if(diferential<0):
            direction = -1
        elif(diferential>0):
            direction = 1
        else:
            direction = 0

        if(abs(diferential)>self.param_velocity):
            diferential=self.param_velocity*direction

        self.armsData.mtfl = diferential*self.param_direction_mtfl

        print "Direccion FL: " + str(diferential*self.param_direction_mtfl)
        print "Posicion actual FL: " + str(self.actual_pos_mtfl)
        print "Posicion deseada FL: " + str(self.desired_pos_mtfl)

        # print "Direccion:" + str(diferential)
        # print "Posicion actual" + str(self.armsActualPos[1])
        # print "Posicion deseada" + str(self.armsDesiredPos[1])

    def updateMtabr(self):
        direction = 0
        diferential = self.desired_pos_mtbr - self.actual_pos_mtbr

        if(diferential<0):
            direction = -1
        elif(diferential>0):
            direction = 1
        else:
            direction = 0

        if(abs(diferential)>self.param_velocity):
            diferential=self.param_velocity*direction

        self.armsData.mtbr = diferential*self.param_direction_mtbr

        # print "Direccion:" + str(diferential)
        # print "Posicion actual" + str(self.armsActualPos[2])
        # print "Posicion deseada" + str(self.armsDesiredPos[2])

        print "Direccion BR: " + str(diferential*self.param_direction_mtbr)
        print "Posicion actual BR: " + str(self.actual_pos_mtbr)
        print "Posicion deseada BR: " + str(self.desired_pos_mtbr)

    def updateMtabl(self):
        direction = 0
        # diferential = self.armsDesiredPos[3] - self.armsActualPos[3]
        diferential = self.desired_pos_mtbl - self.actual_pos_mtbl

        if(diferential<0):
            direction = -1
        elif(diferential>0):
            direction = 1
        else:
            direction = 0

        if(abs(diferential)>self.param_velocity):
            diferential=self.param_velocity*direction

        self.armsData.mtbl = diferential*self.param_direction_mtbl

        #print "Direccion:" + str(diferential*self.param_direction_mtbl)
        #print "Posicion actual" + str(self.actual_pos_mtbl)
        #print "Posicion deseada" + str(self.desired_pos_mtbl)

        print "Direccion BL: " + str(diferential*self.param_direction_mtbl)
        print "Posicion actual BL: " + str(self.actual_pos_mtbl)
        print "Posicion deseada BL: " + str(self.desired_pos_mtbl)


    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # self.update()
            self.updateMtafr()
            self.updateMtafl()
            self.updateMtabr()
            self.updateMtabl()
            self.armsPub.publish(self.armsData)
            # print self.armsActualPos[0]
            # print self.armsActualPos[1]
            # print self.armsActualPos[2]
            # print self.armsActualPos[3]

            r.sleep()

if __name__ == '__main__':
    """ main """
    interface_litarms = Interface_litarms()
    interface_litarms.spin()
    

