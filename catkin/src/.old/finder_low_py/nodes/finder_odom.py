#!/usr/bin/env python
# -*- coding: utf8 -*-

"""
Some code borrowed from diff_tf.py by Jon Stephan
Copyright (C) 2013 Jakob Culebro Reyes
You know the GNU stuff, attribution, no warranty, etc.
"""

import rospy
import roslib
roslib.load_manifest('finder_low_py')

from numpy import array
from std_msgs.msg import Float32MultiArray

from math import sin, cos, pi

from std_msgs.msg import Bool
from std_msgs.msg import UInt16
from std_msgs.msg import Float32

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster


class Finder_odom:
    
    def __init__(self, node_name_in_odo = "finder_odom"):
        rospy.init_node(node_name_in_odo)
        self.nodename = rospy.get_name()
        rospy.loginfo("%s starting" % self.nodename) 
        # parameters, last two are required by the ROS odometry transform, wich may
        # come in handy in the future
        self.rate = rospy.get_param('finder_odom_rate',20.0)
        self.t_delta = rospy.Duration(1. / self.rate)
        self.wheel_radii = rospy.get_param('wheel_radii', 0.254)
        self.base_width = rospy.get_param('base_width', 0.508)
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame 
        # internal variables, r is the LINEAR advance (like the radii of the x, y displacement)
        self.x = 0
        self.y = 0
        self.th = 0
        self.dr = 0
        self.dth = 0
        # suscribers and publishers, refer to the uC node spec
        self.WsSub = rospy.Subscriber("ws", Float32MultiArray, self.wsCallback)
        self.TsPub = rospy.Publisher("Ts", UInt16)
        self.ParRdPub = rospy.Publisher("par_rd", Float32);
        self.ParWdPub = rospy.Publisher("par_wd", Float32);
        self.MePub = rospy.Publisher("Me", Bool)
        # self output
        self.odomPub = rospy.Publisher("odom", Odometry)
        self.odomBroadcaster = TransformBroadcaster()
        
        rospy.loginfo("%s initialized" % self.nodename)
        
        
    def initNodes(self):
        # sync the parameters in the uC
        self.TsPub.publish(self.t_delta)
        self.ParRdPub.publish(self.wheel_radii)
        self.ParWdPub.publish(self.base_width)
        self.MePub.publish(False)
        

    def update(self):
        now = rospy.Time.now()                        
        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2.)
        quaternion.w = cos(self.th / 2.)
        self.odomBroadcaster.sendTransform(
            (self.x, self.y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            rospy.Time.now(),
            self.base_frame_id,
            self.odom_frame_id
            )
        
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dr
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.dth
        self.odomPub.publish(odom)


    def spin(self):
        self.initNodes()
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


    def wsCallback(self, msg):    
        # should be cleaned, also, I'm not dealing with the special case for when
        # the encoder output is published witout any processing
        
        """
        Me:
        ws.data[0] = motorR.encoder.dataADC.lecture;
        ws.data[1] = motorR.encoder.dataADC.timeStamp;
        ws.data[2] = motorL.encoder.dataADC.lecture;
        ws.data[3] = motorL.encoder.dataADC.timeStamp;
        !Me:
        ws.data[0] = motorR.encoder.sum_encoderOutput.delta_phi;
        ws.data[1] = motorR.encoder.sum_encoderOutput.delta_t;
        ws.data[2] = motorL.encoder.sum_encoderOutput.delta_phi;
        ws.data[3] = motorL.encoder.sum_encoderOutput.delta_t;
        """ 
        
        self.p_right = msg.data[0]
        self.p_left = msg.data[2]
        
        # average timeStamp, should soften final result values a little bit. Both
        # timeStamps should be close to equal anyway
        self.d_time = (msg.data[1] + msg.data[3]) / 2.
        
        self.w_right = self.p_right / self.d_time
        self.w_left = self.p_right / self.d_time
        
        self.dr = self.wheel_radii * (self.w_right + self.w_left) / 2.
        self.dth = self.wheel_radii * (self.w_right - self.w_left) / self.base_width
        
        # I'm ussing the diff_ty code, I wonder if this is going to work. We have
        # tested other algorithms before anyway
        self.cth = self.d_time * self.dth
        self.cx = self.d_time * self.dr * cos(self.th + 0.5 * self.dth * self.d_time)
        self.cy = - self.d_time * self.dr * sin(self.th + 0.5 * self.dth * self.d_time)
        
        self.x = self.x + (cos(self.th) * self.cx - sin(self.th) * self.cy)
        self.y = self.y + (sin(self.th) * self.cx + cos(self.th) * self.cy)
        self.th = self.th + self.cth
        

if __name__ == '__main__':
    """ main """
    finder_odom = Finder_odom()
    finder_odom.spin()
    
    
    
