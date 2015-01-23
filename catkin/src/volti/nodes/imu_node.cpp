#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <sstream>

#include <iostream>
#include "imu_lib.h"
#include <cmath>

//#include "volti/float32_3.h"

using namespace std;

float pi= 3.1415926535897;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_node");
  ros::NodeHandle n;
  //ros::Publisher imu_pub = n.advertise<volti::float32_3>("i1", 1000);
  ros::Publisher imu_pub = n.advertise<std_msgs::Float32>("i1", 1000);
  ros::Rate loop_rate(50);
  int count = 0;
  IMU MyImu;
  //volti::float32_3 msg;
  std_msgs::Float32 msg;

  while (ros::ok())
  {
    MyImu.loop();
    //msg.data[0] = MyImu.pitch*180.0/pi;
    //msg.data[0] = MyImu.roll*180.0/pi;
    //msg.data[0] = MyImu.yaw*180.0/pi;
    msg.data = MyImu.accel[0];
    imu_pub.publish(msg);
    msg.data = MyImu.accel[1];
    imu_pub.publish(msg);
    msg.data = MyImu.accel[2];
    imu_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}