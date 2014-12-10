#!/usr/bin/env python
# -*- coding: utf8 -*-

import roslib; roslib.load_manifest('razor_imu_9dof')
import rospy
import math
import tf

from time import time
from sensor_msgs.msg import Imu
from finder.msg import float32_12

class Imu_input:

    def __init__(self, node_name_override = 'imu_input'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("imu_input starting with name %s", self.inputname) 
        self.rate = rospy.get_param("param_global_rate", 10)
        self.init_time = rospy.get_time()

        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.grad2rad = 3.141592/180.0
        self.imuPub = rospy.Publisher('imu_topic', Imu)
        self.imuMsg = Imu()
        self.imuMsg.orientation_covariance = [999999 , 0 , 0, 0, 9999999, 0, 0, 0, 999999]
        self.imuMsg.angular_velocity_covariance = [9999, 0 , 0, 0 , 99999, 0, 0 , 0 , 0.02]
        self.imuMsg.linear_acceleration_covariance = [0.2 , 0 , 0, 0 , 0.2, 0, 0 , 0 , 0.2]

        self.imuSub = rospy.Subscriber("imu_data", float32_12, self.imuDataCb)

        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.q = tf.transformations.quaternion_from_euler(self.roll,self.pitch,self.yaw)
        self.imuMsg.orientation.x = self.q[0]
        self.imuMsg.orientation.y = self.q[1]
        self.imuMsg.orientation.z = self.q[2]
        self.imuMsg.orientation.w = self.q[3]
        self.imuMsg.header.stamp= rospy.Time.now()
        self.imuMsg.header.frame_id = 'base_link'
        self.imuPub.publish(self.imuMsg)

    def imuDataCb(self, data):

        self.yaw = 0 * data.data[0] * self.grad2rad
        self.pitch = 0 * data.data[1] * self.grad2rad
        self.roll = 0 * data.data[2] * self.grad2rad
            
        self.q = tf.transformations.quaternion_from_euler(self.roll,self.pitch,self.yaw)
        
        self.imuMsg.orientation.x = self.q[0]
        self.imuMsg.orientation.y = self.q[1]
        self.imuMsg.orientation.z = self.q[2]
        self.imuMsg.orientation.w = self.q[3]
        self.imuMsg.header.stamp= rospy.Time.now()
        self.imuMsg.header.frame_id = 'base_link'
        
    def update(self):

        self.imuMsg.header.stamp = rospy.Time.now()
        self.imuPub.publish(self.imuMsg)

    def spin(self):

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

if __name__ == '__main__':

    """ main """
    imu_input = Imu_input()
    imu_input.spin()