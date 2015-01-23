#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <iostream>
#include "imu_lib.h"
#include <cmath>

#include "volti/float32_3.h"

using namespace std;

float pi= 3.1415926535897;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_node");
  ros::NodeHandle n;
  ros::Publisher imu_pub = n.advertise<volti::float32_3>("i1", 1000);
  ros::Rate loop_rate(50);
  int count = 0;
  IMU MyImu;
  volti::float32_3 msg;

  while (ros::ok())
  {
    MyImu.loop();
    msg.data[0] = MyImu.pitch*180.0/pi
    msg.data[0] = MyImu.roll*180.0/pi
    msg.data[0] = MyImu.yaw*180.0/pi
    imu_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}