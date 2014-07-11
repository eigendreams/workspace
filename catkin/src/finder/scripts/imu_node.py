#!/usr/bin/env python

# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('razor_imu_9dof')
import rospy
import math
from time import time
from sensor_msgs.msg import Imu
import tf
from finder.msg import float32_12

class Imu_node:

    def __init__(self, node_name_override = 'imu_node'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("imu_node starting with name %s", self.nodename) 
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


    def imuDataCb(self, data):

        self.yaw = data.data[0] * self.grad2rad
        self.pitch = data.data[1] * self.grad2rad
        self.roll = data.data[2] * self.grad2rad
            
        self.q = tf.transformations.quaternion_from_euler(self.roll,self.pitch,self.yaw)
        
        self.imuMsg.orientation.x = self.q[0]
        self.imuMsg.orientation.y = self.q[1]
        self.imuMsg.orientation.z = self.q[2]
        self.imuMsg.orientation.w = self.q[3]
        self.imuMsg.header.stamp= rospy.Time.now()
        self.imuMsg.header.frame_id = 'base_link'
        self.imuPub.publish(self.imuMsg)

    def update(self):

        pass

    def spin(self):

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

if __name__ == '__main__':

    """ main """
    imu_node = Imu_node()
    imu_node.spin()