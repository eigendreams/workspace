#!/usr/bin/env python

import roslib; roslib.load_manifest('volti')
import rospy
import math
from   time         import time
from   volti.msg    import float32_3
from   volti.msg    import float32_12
from   numpy        import *
from   numpy.linalg import inv 

grad2rad = 3.141592/180.0

R_IDEM = array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
R_roll_180 = array([[1, 0, 0],[0, cos(3.1416), -sin(3.1416)],[0, sin(3.1416), cos(3.1416)]])
R_pitch_180 = array([[cos(3.1416), 0, sin(3.1416)],[0, 1, 0],[-sin(3.1416), 0, cos(3.1416)]])

rospy.init_node("top_control_node")

def processIMU1_message(rawMsg):

    roll = rawMsg.data[0]
    pitch = rawMsg.data[1]
    yaw = rawMsg.data[2]

    R_yaw_z = array([[cos(yaw), sin(yaw), 0], [-sin(yaw), cos(yaw), 0], [0, 0, 1]])
    R_pitch_y = array([[cos(pitch), 0, sin(pitch)],[0, 1, 0],[-sin(pitch), 0, cos(pitch)]])
    R_roll_x = array([[1, 0, 0],[0, cos(roll), -sin(roll)],[0, sin(roll), cos(roll)]])
   
    x_axis_imu = (dot(R_yaw_z, dot(R_pitch_y, dot(R_roll_x, array([[1],[0],[0]])))))
    y_axis_imu = (dot(R_yaw_z, dot(R_pitch_y, dot(R_roll_x, array([[0],[1],[0]])))))
    z_axis_imu = (dot(R_yaw_z, dot(R_pitch_y, dot(R_roll_x, array([[0],[0],[1]])))))
    
    x_test.axis = x_axis_imu
    y_test.axis = y_axis_imu
    z_test.axis = z_axis_imu

def processIMU2_message(rawMsg):

    roll2 = rawMsg.data[0]
    pitch2 = rawMsg.data[1]
    yaw2 = rawMsg.data[2]

    R_yaw_z2 = array([[cos(yaw2), sin(yaw2), 0], [-sin(yaw2), cos(yaw2), 0], [0, 0, 1]])
    R_pitch_y2 = array([[cos(pitch2), 0, sin(pitch2)],[0, 1, 0],[-sin(pitch2), 0, cos(pitch2)]])
    R_roll_x2 = array([[1, 0, 0],[0, cos(roll2), -sin(roll2)],[0, sin(roll2), cos(roll2)]])

    x_axis_imu2 = dot(R_yaw_z2, dot(R_pitch_y2, dot(R_roll_x2, array([[1],[0],[0]]))))
    y_axis_imu2 = dot(R_yaw_z2, dot(R_pitch_y2, dot(R_roll_x2, array([[0],[1],[0]]))))
    z_axis_imu2 = dot(R_yaw_z2, dot(R_pitch_y2, dot(R_roll_x2, array([[0],[0],[1]]))))

    x_test_2.axis = x_axis_imu2
    y_test_2.axis = y_axis_imu2
    z_test_2.axis = z_axis_imu2

sub1 = rospy.Subscriber('imu_plate_3', float32_3, processIMU1_message)
sub2 = rospy.Subscriber('imu_pendu_3', float32_3, processIMU2_message)