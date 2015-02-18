#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <sstream>

#include <iostream>
#include "imu_lib.h"
#include <cmath>

#include "volti/float32_3.h"

using namespace std;

float pi= 3.1415926535897;

///////////////////////////////////////////////////////////////////////////////

class Kalman1D {

public:

  Kalman1D() {
    times = 0;
    dt = 0.020;
  }

  void transpose(float destination[2][2], float A[2][2]) {
    destination[0][0] = A[0][0];
    destination[0][1] = A[1][0];
    destination[1][0] = A[0][1];
    destination[1][1] = A[1][1];
  }

  void inverse(float destination[2][2], float A[2][2]) {
    float a = A[0][0];
    float c = A[1][0];
    float b = A[0][1];
    float d = A[1][1];

    float divisor = (a*d - b * c);

    destination[0][0] = d / divisor;
    destination[1][0] = -c / divisor;
    destination[0][1] = -b / divisor;
    destination[1][1] = a / divisor;
  }

  void minusVectorVector(float destination[2][1], float A[2][1], float B[2][1]) {
    destination[0][0] = A[0][0] - B[0][0];
    destination[1][0] = A[1][0] - B[1][0];
  }

  void sumVectorVector(float destination[2][1], float A[2][1], float B[2][1]) {
    destination[0][0] = A[0][0] + B[0][0];
    destination[1][0] = A[1][0] + B[1][0];
  }

  void minusMatrixMatrix(float destination[2][2], float A[2][2], float B[2][2]) {
    destination[0][0] = A[0][0] - B[0][0];
    destination[0][1] = A[0][1] - B[0][1];
    destination[1][0] = A[1][0] - B[1][0];
    destination[1][1] = A[1][1] - B[1][1];
  }

  void sumMatrixMatrix(float destination[2][2], float A[2][2], float B[2][2]) {
    destination[0][0] = A[0][0] + B[0][0];
    destination[0][1] = A[0][1] + B[0][1];
    destination[1][0] = A[1][0] + B[1][0];
    destination[1][1] = A[1][1] + B[1][1];
  }

  void copyVector(float destination[2][1], float origin[2][1]) {
    destination[0][0] = origin[0][0];
    destination[1][0] = origin[1][0];
  }

  void copyMatrix(float destination[2][2], float origin[2][2]) {
    destination[0][0] = origin[0][0];
    destination[0][1] = origin[0][1];
    destination[1][0] = origin[1][0];
    destination[1][1] = origin[1][1];
  }

  void multMatrixMatrix(float destination[2][2], float A[2][2], float B[2][2] ) {
    destination[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0];
    destination[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1];
    destination[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0];
    destination[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1];
  }

  void multMatrixVector(float destination[2][1], float A[2][2], float V[2][1]) {
    destination[0][0] = A[0][0] * V[0][0] + A[0][1] * V[1][0];
    destination[1][0] = A[1][0] * V[0][0] + A[1][1] * V[1][0];
  }

  float multVectorVector(float V1[2][1], float V2[2][1]) {
    return ((V1[0][0]) * (V2[0][0]) + (V1[1][0]) * (V2[1][0]));
  }

  float A [2][2]      = {{1, dt}, {0, 1}};
  float H [2][2]      = {{1, 0}, {0, 1}};
  float Y [2][1]      = {{0}, {0}};
  float Ym1 [2][1]    = {{0}, {0}};
  float Ym1m1 [2][1]  = {{0}, {0}};
  float Xkp [2][1]    = {{0}, {0}};
  float Xk[2][1]      = {{0}, {0}};
  float Xkm1 [2][1]   = {{0}, {0}};
  float Xkm1m1 [2][1] = {{0}, {0}};
  float Q[2][2]       = {{10, 0}, {0, 10}};
  float R[2][2]       = {{10, 0}, {0, 10}};
  float Pkp [2][2]    = {{10, 0}, {0, 10}};
  float Pk[2][2]      = {{0, 0}, {0, 0}};
  float Pkm1 [2][2]   = {{0, 0}, {0, 0}};
  float Vk [2][1]     = {{0}, {0}};
  float Sk [2][2]     = {{0, 0}, {0, 0}};
  float Kk [2][2]     = {{0, 0}, {0, 0}};

  float TM1 [2][2]    = {{0, 0}, {0, 0}};
  float TM2 [2][2]    = {{0, 0}, {0, 0}};
  float TM3 [2][2]    = {{0, 0}, {0, 0}};
  float TV1 [2][1]    = {{0}, {0}};
  float TV2 [2][1]    = {{0}, {0}};
  float TV3 [2][1]    = {{0}, {0}};

  long times;
  float dt;

  void compute(float measure) {

    times++;

    if (times < 4) {

      float startV [2][1] = {{measure}, {0}};

      copyVector(Y, startV);
      copyVector(Ym1, startV);
      copyVector(Ym1m1, startV);
      copyVector(Xk, startV);
      copyVector(Xkm1, startV);
      copyVector(Xkm1m1, startV);
    }

    copyVector(Ym1m1, Ym1);
    copyVector(Ym1, Y);
    copyVector(Xkm1m1, Xkm1);
    copyVector(Xkm1, Xk);


    multMatrixVector(Xkp, A, Xkm1);


    transpose(TM1, A);
    multMatrixMatrix(TM2, Pkm1, TM1);
    multMatrixMatrix(TM3, A, TM2);
    sumMatrixMatrix(Pkp, TM3, Q);


    Y[0][0] = measure;
    Y[1][0] = (measure - Ym1[1][0]) / dt;
    Y[2][0] = (Ym1[1][0] - Ym1m1[1][0]) / dt;


    multMatrixVector(TV1, H, Xkp);
    minusVectorVector(Vk, Y, TV1);


    transpose(TM1, H);
    multMatrixMatrix(TM2, Pkp, TM1);
    multMatrixMatrix(TM3, H, TM2);
    sumMatrixMatrix(Sk, TM3, R);


    transpose(TM1, H);
    inverse(TM2, Sk);
    multMatrixMatrix(TM3, TM1, TM2);
    multMatrixMatrix(Kk, Pkp, TM3);

    multMatrixVector(TV1, Kk, Vk);
    sumVectorVector(Xk, Xkp, TV1);

    transpose(TM1, Kk);
    multMatrixMatrix(TM2, Sk, TM1);
    multMatrixMatrix(TM3, Kk, TM2);
    minusMatrixMatrix(Pk, Pkp, TM3);

    copyMatrix(Pkm1, Pk);
  }

  float get() {
    return Xk[0][0];
  }

};
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_node");
  ros::NodeHandle n;

  ros::Publisher imu1_pub = n.advertise<volti::float32_3>("imu_pendu_3", 1);
  ros::Publisher imu2_pub = n.advertise<volti::float32_3>("imu_plate_3", 1);

  //publicar cada 20 ms
  ros::Rate loop_rate(50);

  // modulos de salida en el BBB
  IMU imu_plate(1);
  IMU imu_pendu(2);

  volti::float32_3 imu_plate_msg;
  volti::float32_3 imu_pendu_msg;

  Kalman1D kalman1;
  Kalman1D kalman2;
  Kalman1D kalman3;
  Kalman1D kalman4;
  Kalman1D kalman5;
  Kalman1D kalman6;

  while (ros::ok())
  {
    imu_plate.loop();

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

    imu_pendu.loop();

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
    
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}