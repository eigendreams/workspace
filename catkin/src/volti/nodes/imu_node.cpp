#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <sstream>

#include <iostream>
#include "imu_lib.h"
#include <cmath>

#include "volti_msgs/float32_3.h"

using namespace std;

float pi= 3.1415926535897;

///////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_node");
  ros::NodeHandle n;

  ros::Publisher imu1_pub = n.advertise<volti_msgs::float32_3>("imu_pendu_3", 1);
  ros::Publisher imu2_pub = n.advertise<volti_msgs::float32_3>("imu_plate_3", 1);

  //publicar cada 20 ms
  ros::Rate loop_rate(20);

  // modulos de salida en el BBB
  IMU imu_plate(1);
  IMU imu_pendu(2);

  volti_msgs::float32_3 imu_plate_msg;
  volti_msgs::float32_3 imu_pendu_msg;

  int unsigned long times = 0;

  while (ros::ok())
  {
    imu_plate.loop();
    imu_pendu.loop();

    times++;

    if (times % 2 == 1) {

    //kalman1.compute(imu_plate.roll);
    //kalman2.compute(imu_plate.pitch);
    //kalman3.compute(imu_plate.yaw);

    //imu_plate_msg.data[0] = kalman1.get();
    //imu_plate_msg.data[1] = kalman2.get();
    //imu_plate_msg.data[2] = kalman3.get();
    imu_plate_msg.data[0] = imu_plate.roll;
    imu_plate_msg.data[1] = imu_plate.pitch;
    imu_plate_msg.data[2] = imu_plate.yaw;
    /*imu_plate_msg.data[3] = imu_plate.accel[0] * 9.806 / 256.0;
    imu_plate_msg.data[4] = imu_plate.accel[1] * 9.806 / 256.0;
    imu_plate_msg.data[5] = imu_plate.accel[2] * 9.806 / 256.0;
    imu_plate_msg.data[6] = imu_plate.magnetom[0] * 1;
    imu_plate_msg.data[7] = imu_plate.magnetom[1] * 1;
    imu_plate_msg.data[8] = imu_plate.magnetom[2] * 1;
    imu_plate_msg.data[9] = imu_plate.gyro[0] * 0.06957 * 0.01745329252;
    imu_plate_msg.data[10] = imu_plate.gyro[1] * 0.06957 * 0.01745329252;
    imu_plate_msg.data[11] = imu_plate.gyro[2] * 0.06957 * 0.01745329252;*/
    imu1_pub.publish(imu_plate_msg);

    

    //kalman4.compute(imu_pendu.roll);
    //kalman5.compute(imu_pendu.pitch);
    //kalman6.compute(imu_pendu.yaw);

    //imu_pendu_msg.data[0] = kalman4.get();
    //imu_pendu_msg.data[1] = kalman5.get();
    //imu_pendu_msg.data[2] = kalman6.get();
    imu_pendu_msg.data[0] = imu_pendu.roll;
    imu_pendu_msg.data[1] = imu_pendu.pitch;
    imu_pendu_msg.data[2] = imu_pendu.yaw;
    /*imu_pendu_msg.data[3] = imu_pendu.accel[0] * 9.806 / 256.0;
    imu_pendu_msg.data[4] = imu_pendu.accel[1] * 9.806 / 256.0;
    imu_pendu_msg.data[5] = imu_pendu.accel[2] * 9.806 / 256.0;
    imu_pendu_msg.data[6] = imu_pendu.magnetom[0] * 1;
    imu_pendu_msg.data[7] = imu_pendu.magnetom[1] * 1;
    imu_pendu_msg.data[8] = imu_pendu.magnetom[2] * 1;
    imu_pendu_msg.data[9] = imu_pendu.gyro[0] * 0.06957 * 0.01745329252;
    imu_pendu_msg.data[10] = imu_pendu.gyro[1] * 0.06957 * 0.01745329252;
    imu_pendu_msg.data[11] = imu_pendu.gyro[2] * 0.06957 * 0.01745329252;*/
    imu2_pub.publish(imu_pendu_msg);

    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}