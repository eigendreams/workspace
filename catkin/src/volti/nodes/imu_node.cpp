#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <sstream>

#include <iostream>
#include "imu_lib.h"
#include <cmath>

#include "volti/float32_12.h"
//#include "volti/float32_3.h"

using namespace std;

float pi= 3.1415926535897;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_node");
  ros::NodeHandle n;
  ros::Publisher imu1_pub = n.advertise<volti::float32_12>("i1", 1000);
  ros::Publisher imu2_pub = n.advertise<volti::float32_12>("i2", 1000);
  ros::Rate loop_rate(50);
  IMU MyImu1(1);
  IMU MyImu2(2);
  volti::float32_12 msg1;
  volti::float32_12 msg2;

  while (ros::ok())
  {
    MyImu1.loop();

    msg1.data[0] = MyImu1.yaw * 180 / pi;
    msg1.data[1] = MyImu1.pitch * 180 / pi;
    msg1.data[2] = MyImu1.roll * 180 / pi;
    msg1.data[3] = MyImu1.accel[0];
    msg1.data[4] = MyImu1.accel[1];
    msg1.data[5] = MyImu1.accel[2];
    msg1.data[6] = MyImu1.magnetom[0];
    msg1.data[7] = MyImu1.magnetom[1];
    msg1.data[8] = MyImu1.magnetom[2];
    msg1.data[9] = MyImu1.gyro[0];
    msg1.data[10] = MyImu1.gyro[1];
    msg1.data[11] = MyImu1.gyro[2];
    imu1_pub.publish(msg1);

    MyImu2.loop();

    msg2.data[0] = MyImu2.yaw * 180 / pi;
    msg2.data[1] = MyImu2.pitch * 180 / pi;
    msg2.data[2] = MyImu2.roll * 180 / pi;
    msg2.data[3] = MyImu2.accel[0];
    msg2.data[4] = MyImu2.accel[1];
    msg2.data[5] = MyImu2.accel[2];
    msg2.data[6] = MyImu2.magnetom[0];
    msg2.data[7] = MyImu2.magnetom[1];
    msg2.data[8] = MyImu2.magnetom[2];
    msg2.data[9] = MyImu2.gyro[0];
    msg2.data[10] = MyImu2.gyro[1];
    msg2.data[11] = MyImu2.gyro[2];
    imu2_pub.publish(msg2);
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}