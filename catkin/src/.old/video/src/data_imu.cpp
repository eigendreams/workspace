/*
 * data_imu.cpp
 *
 *  Created on: 09/11/2012
 *      Author: raul
 */

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Float32MultiArray.h"


class Manejador{
	public:
		Manejador();
	private:
		void func(const geometry_msgs::Vector3::ConstPtr& angs);
		ros::NodeHandle node;
		ros::Publisher pub;
		ros::Subscriber sub;
};



Manejador::Manejador(){
	pub=node.advertise<sensor_msgs::Imu>("angulos_imu",10);
	sub=node.subscribe<geometry_msgs::Vector3>("ypr",10,&Manejador::func,this);
}

void Manejador::func(const geometry_msgs::Vector3::ConstPtr& YPR){
	geometry_msgs::Quaternion rotacion=tf::createQuaternionMsgFromRollPitchYaw(YPR->z,YPR->y,YPR->x);
	
	sensor_msgs::Imu imu;
	imu.header.frame_id="frame_imu";
	imu.header.stamp=ros::Time::now();
	imu.orientation=rotacion;
	imu.orientation_covariance[0]=10;
	imu.orientation_covariance[4]=10;
	imu.orientation_covariance[8]=10;
	imu.angular_velocity_covariance[0]=-1;
	imu.linear_acceleration_covariance[0]=-1;
	pub.publish(imu);
}


int main(int argc, char** argv){
	ros::init(argc, argv,"data_imu");
	Manejador m;
	ros::spin();
	return 0;
}
