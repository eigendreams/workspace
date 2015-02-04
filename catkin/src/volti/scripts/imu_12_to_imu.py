#!/usr/bin/env python

import roslib; roslib.load_manifest('volti')
import rospy
import math

from volti.msg import float32_12
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import quaternion_from_euler

rospy.init_node("imu_12_to_imu")

pub_imu_plate     = rospy.Publisher('imu_plate',     Imu, queue_size = 5)
pub_imu_pendu     = rospy.Publisher('imu_pendu',     Imu, queue_size = 5)
pub_imu_plate_mag = rospy.Publisher('imu_plate_mag', Imu, queue_size = 5)
pub_imu_pendu_mag = rospy.Publisher('imu_pendu_mag', Imu, queue_size = 5)

imu_plate_Msg     = Imu()
imu_plate_mag_Msg = Vector3Stamped()
imu_pendu_Msg     = Imu()
imu_pendu_mag_Msg = Vector3Stamped()

imu_plate_Msg.orientation_covariance = [
0.0025 , 0 , 0,
0, 0.0025, 0,
0, 0, 0.0025
]

imu_plate_Msg.angular_velocity_covariance = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

imu_plate_Msg.linear_acceleration_covariance = [
0.04 , 0 , 0,
0 , 0.04, 0,
0 , 0 , 0.04
]

imu_pendu_Msg.orientation_covariance = [
0.0025 , 0 , 0,
0, 0.0025, 0,
0, 0, 0.0025
]

imu_pendu_Msg.angular_velocity_covariance = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

imu_pendu_Msg.linear_acceleration_covariance = [
0.04 , 0 , 0,
0 , 0.04, 0,
0 , 0 , 0.04
]

def imu_plate_12_cb(rawMsg):
    #
    roll_imu_plate  = float(rawMsg.data[0])
    pitch_imu_plate = float(rawMsg.data[1])
    yaw_imu_plate   = float(rawMsg.data[2])
    #
    q_imu_plate = quaternion_from_euler(roll_imu_plate, pitch_imu_plate, yaw_imu_plate)
    #    
    imu_plate_Msg.orientation.x = q[0]
    imu_plate_Msg.orientation.y = q[1]
    imu_plate_Msg.orientation.z = q[2]
    imu_plate_Msg.orientation.w = q[3]
    #
    imu_plate_Msg.linear_acceleration.x = float(rawMsg.data[3]) * 9.806 / 256.0 
    imu_plate_Msg.linear_acceleration.y = float(rawMsg.data[4]) * 9.806 / 256.0 
    imu_plate_Msg.linear_acceleration.z = float(rawMsg.data[5]) * 9.806 / 256.0 
    #
    imu_plate_Msg.angular_velocity.x = float(rawMsg.data[9]) * 0.01745329252 * 0.06957
    imu_plate_Msg.angular_velocity.y = float(rawMsg.data[10]) * 0.01745329252 * 0.06957
    imu_plate_Msg.angular_velocity.z = float(rawMsg.data[11]) * 0.01745329252 * 0.06957
    #
    imu_plate_Msg.header.stamp = rospy.Time.now()
    imu_plate_Msg.header.frame_id = 'base'
    pub_imu_plate.publish(imu_plate_Msg)
    #
    imu_plate_mag_Msg.vector.x = float(rawMsg.data[6])
    imu_plate_mag_Msg.vector.y = float(rawMsg.data[7])
    imu_plate_mag_Msg.vector.z = float(rawMsg.data[8])

    imu_plate_mag_Msg.header.stamp = rospy.Time.now()
    imu_plate_mag_Msg.header.frame_id = 'base'
    pub_imu_plate_mag.publish(imu_plate_mag_Msg)
    #
def imu_pendu_12_cb(rawMsg):
    #
    roll_imu_pendu  = float(rawMsg.data[0])
    pitch_imu_pendu = float(rawMsg.data[1])
    yaw_imu_pendu   = float(rawMsg.data[2])
    #
    q_imu_pendu = quaternion_from_euler(roll_imu_pendu, pitch_imu_pendu, yaw_imu_pendu)
    #    
    imu_pendu_Msg.orientation.x = q[0]
    imu_pendu_Msg.orientation.y = q[1]
    imu_pendu_Msg.orientation.z = q[2]
    imu_pendu_Msg.orientation.w = q[3]
    #
    imu_pendu_Msg.linear_acceleration.x = float(rawMsg.data[3]) * 9.806 / 256.0 
    imu_pendu_Msg.linear_acceleration.y = float(rawMsg.data[4]) * 9.806 / 256.0 
    imu_pendu_Msg.linear_acceleration.z = float(rawMsg.data[5]) * 9.806 / 256.0 
    #
    imu_pendu_Msg.angular_velocity.x = float(rawMsg.data[9]) * 0.01745329252 * 0.06957
    imu_pendu_Msg.angular_velocity.y = float(rawMsg.data[10]) * 0.01745329252 * 0.06957
    imu_pendu_Msg.angular_velocity.z = float(rawMsg.data[11]) * 0.01745329252 * 0.06957
    #
    imu_pendu_Msg.header.stamp = rospy.Time.now()
    imu_pendu_Msg.header.frame_id = 'base'
    pub_imu_pendu.publish(imu_pendu_Msg)
    #
    imu_pendu_mag_Msg.vector.x = float(rawMsg.data[6])
    imu_pendu_mag_Msg.vector.y = float(rawMsg.data[7])
    imu_pendu_mag_Msg.vector.z = float(rawMsg.data[8])

    imu_pendu_mag_Msg.header.stamp = rospy.Time.now()
    imu_pendu_mag_Msg.header.frame_id = 'base'
    pub_imu_pendu_mag.publish(imu_pendu_mag_Msg)
    #
sub_imu_plate_12 = rospy.Subscriber("imu_plate_12", float32_12, imu_plate_12_cb)
sub_imu_pendu_12 = rospy.Subscriber("imu_pendu_12", float32_12, imu_pendu_12_cb)

rospy.spin()