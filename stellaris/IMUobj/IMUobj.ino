#include "Arduino.h"
#include "ros.h"
#include "Servo.h"
#include "finder/float32_12.h"
#include "Wire.h"
#include "IMU.h"

ros::NodeHandle nh;
long timelast = 0;
finder::float32_12 imu_data1;
ros::Publisher imu_pub1("imu_data1", &imu_data1);
IMU imu1(3);
finder::float32_12 imu_data2;
ros::Publisher imu_pub2("imu_data2", &imu_data2);
IMU imu2(2);

Servo servi;

void setup()
{
  // Init serial output
  //Serial.begin(OUTPUT__BAUD_RATE);
  //nh.initNode();
  //nh.advertise(imu_pub1);
  //nh.advertise(imu_pub2);
  
  //imu1.setup(); 
  //imu2.setup(); 

  servi.attach(13);
  pinMode(13, OUTPUT);
}

int ang = 0;

// Main loop
void loop()
{
  //imu1.loop();
  //imu2.loop();

  long timenow = millis();

  if (timenow - timelast >= 100) {

    ang = ang + 10;
    ang = constrain(ang, 0, 1000);
    if (ang > 999) ang = 0;
    servi.write(ang + 1000);

    timelast = timenow;
    
    /*imu_data1.data[0] = TO_DEG(imu1.yaw);
    imu_data1.data[1] = TO_DEG(imu1.pitch);
    imu_data1.data[2] = TO_DEG(imu1.roll);

    imu_data1.data[3] = imu1.accel[0];
    imu_data1.data[4] = imu1.accel[1];
    imu_data1.data[5] = imu1.accel[2];

    imu_data1.data[6] = imu1.magnetom[0];
    imu_data1.data[7] = imu1.magnetom[1]; 
    imu_data1.data[8] = imu1.magnetom[2];

    imu_data1.data[9] = imu1.gyro[0];
    imu_data1.data[10] = imu1.gyro[1];
    imu_data1.data[11] = imu1.gyro[2];

    imu_pub1.publish(&imu_data1);

    imu_data2.data[0] = TO_DEG(imu2.yaw);
    imu_data2.data[1] = TO_DEG(imu2.pitch);
    imu_data2.data[2] = TO_DEG(imu2.roll);

    imu_data2.data[3] = imu2.accel[0];
    imu_data2.data[4] = imu2.accel[1];
    imu_data2.data[5] = imu2.accel[2];

    imu_data2.data[6] = imu2.magnetom[0];
    imu_data2.data[7] = imu2.magnetom[1]; 
    imu_data2.data[8] = imu2.magnetom[2];

    imu_data2.data[9] = imu2.gyro[0];
    imu_data2.data[10] = imu2.gyro[1];
    imu_data2.data[11] = imu2.gyro[2];

    imu_pub2.publish(&imu_data2);
    
    nh.spinOnce();*/
  }
}
