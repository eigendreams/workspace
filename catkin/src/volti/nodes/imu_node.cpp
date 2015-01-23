#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <sstream>

#include <iostream>
#include "imu_lib.h"
#include <cmath>

#include "volti/float32_12.h"
#include "volti/float32_3.h"

using namespace std;

float pi= 3.1415926535897;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_node");
  ros::NodeHandle n;
  ros::Publisher imu_pub = n.advertise<volti::float32_12>("i1", 1000);
  //ros::Publisher imu_pub = n.advertise<std_msgs::Float32>("i1", 1000);
  ros::Rate loop_rate(50);
  int count = 0;
  IMU MyImu;
  volti::float32_12 msg;
  //std_msgs::Float32 msg;

  while (ros::ok())
  {
    MyImu.loop();
    msg.data[0] = MyImu.yaw * 180 / pi;
    msg.data[1] = MyImu.pitch * 180 / pi;
    msg.data[2] = MyImu.roll * 180 / pi;
    msg.data[3] = MyImu.accel[0];
    msg.data[4] = MyImu.accel[1];
    msg.data[5] = MyImu.accel[2];
    msg.data[6] = MyImu.magnetom[0];
    msg.data[7] = MyImu.magnetom[1];
    msg.data[8] = MyImu.magnetom[2];
    msg.data[9] = MyImu.gyro[0];
    msg.data[10] = MyImu.gyro[1];
    msg.data[11] = MyImu.gyro[2];
    imu_pub.publish(msg);
    //msg.data = MyImu.accel[0];
    //imu_pub.publish(msg);
    //msg.data = MyImu.accel[1];
    //imu_pub.publish(msg);
    //msg.data = MyImu.accel[2];
    //imu_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}