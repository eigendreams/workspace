#include "imu_lib.h"
#include <cmath>
#include <cfloat>
#include <unistd.h>

/* This file is part of the Razor AHRS Firmware */

IMU::IMU(int module)
{
  this->module = module;
  this->output_mode = OUTPUT__MODE_ANGLES;
  this->output_format = OUTPUT__FORMAT_TEXT;
  this->output_errors = false;
  this->gyro_num_samples = 0;
  this->curr_calibration_sensor = 0;
  this->reset_calibration_session_flag = true;
  this->num_accel_errors = 0;
  this->num_magn_errors = 0;
  this->num_gyro_errors = 0;

  this->I2C_Init();

}

void IMU::Compass_Heading()
{
  float mag_x;
  float mag_y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
  // Tilt compensated magnetic field X
  mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch;
  // Tilt compensated magnetic field Y
  mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
  // Magnetic Heading
  float MAG_Heading_TMP = atan2(-mag_y, mag_x);

  if (std::isfinite(MAG_Heading_TMP))
    MAG_Heading = MAG_Heading_TMP;
}

/* This file is part of the Razor AHRS Firmware */

// DCM algorithm

/**************************************************/
void IMU::Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;
  
  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
  
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
  
  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

/**************************************************/
void IMU::Drift_correction(void)
{
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
  //Compensation the Roll, Pitch and Yaw drift. 
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;
  
  
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = this->constrain(1 - 2*std::abs(1.0 - Accel_magnitude),0,1);  //

  Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
  
  Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);     
  
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
 
  mag_heading_x = cos(MAG_Heading);
  mag_heading_y = sin(MAG_Heading);
  errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
  Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  
  Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
  
  Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
}

float IMU::constrain(float X, float A, float B) {
  if(X>=A && X<=B){
    return X;
  }else if(X<A){
    return A;
  }else{
    return B;
  }
}

void IMU::Matrix_update(void)
{
  Gyro_Vector[0]=1.0 * GYRO_SCALED_RAD(gyro[0]); //gyro x roll
  Gyro_Vector[1]=1.0 * GYRO_SCALED_RAD(gyro[1]); //gyro y pitch
  Gyro_Vector[2]=1.0 * GYRO_SCALED_RAD(gyro[2]); //gyro z yaw
  
  Accel_Vector[0]=accel[0];
  Accel_Vector[1]=accel[1];
  Accel_Vector[2]=accel[2];
    
  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term
  
#if DEBUG__NO_DRIFT_CORRECTION == true // Do not use drift correction
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
  Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
  Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
  Update_Matrix[2][2]=0;
#else // Use drift correction
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2]=0;
#endif

  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

  for(short x=0; x<3; x++) //Matrix Addition (update)
  {
    for(short y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    } 
  }
}

void IMU::Euler_angles(void) {
  float pitch_TMP = -asin(DCM_Matrix[2][0]);
  float roll_TMP = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
  float yaw_TMP = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
  //
  pitch = pitch_TMP;
  if (std::isfinite(roll_TMP))
    roll = roll_TMP;
  if (std::isfinite(yaw_TMP))
    yaw = yaw_TMP;
}

/* This file is part of the Razor AHRS Firmware */

// Computes the dot product of two vectors
float IMU::Vector_Dot_Product(const float v1[3], const float v2[3])
{
  float result = 0;
  
  for(short c = 0; c < 3; c++)
  {
    result += v1[c] * v2[c];
  }
  
  return result; 
}

// Computes the cross product of two vectors
// out has to different from v1 and v2 (no in-place)!
void IMU::Vector_Cross_Product(float out[3], const float v1[3], const float v2[3])
{
  out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
  out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
  out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

// Multiply the vector by a scalar
void IMU::Vector_Scale(float out[3], const float v[3], float scale)
{
  for(short c = 0; c < 3; c++)
  {
    out[c] = v[c] * scale; 
  }
}

// Adds two vectors
void IMU::Vector_Add(float out[3], const float v1[3], const float v2[3])
{
  for(short c = 0; c < 3; c++)
  {
    out[c] = v1[c] + v2[c];
  }
}

// Multiply two 3x3 matrices: out = a * b
// out has to different from a and b (no in-place)!
void IMU::Matrix_Multiply(const float a[3][3], const float b[3][3], float out[3][3])
{
  for(short x = 0; x < 3; x++)  // rows
  {
    for(short y = 0; y < 3; y++)  // columns
    {
      out[x][y] = a[x][0] * b[0][y] + a[x][1] * b[1][y] + a[x][2] * b[2][y];
    }
  }
}

// Multiply 3x3 matrix with vector: out = a * b
// out has to different from b (no in-place)!
void IMU::Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3])
{
  for(short x = 0; x < 3; x++)
  {
    out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
  }
}

// Init rotation matrix using euler angles
void IMU::init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll)
{
  float c1 = cos(roll);
  float s1 = sin(roll);
  float c2 = cos(pitch);
  float s2 = sin(pitch);
  float c3 = cos(yaw);
  float s3 = sin(yaw);

  // Euler angles, right-handed, intrinsic, XYZ convention
  // (which means: rotate around body axes Z, Y', X'') 
  m[0][0] = c2 * c3;
  m[0][1] = c3 * s1 * s2 - c1 * s3;
  m[0][2] = s1 * s3 + c1 * c3 * s2;

  m[1][0] = c2 * s3;
  m[1][1] = c1 * c3 + s1 * s2 * s3;
  m[1][2] = c1 * s2 * s3 - c3 * s1;

  m[2][0] = -s2;
  m[2][1] = c2 * s1;
  m[2][2] = c1 * c2;
}

/* This file is part of the Razor AHRS Firmware */


void IMU::I2C_Init()
{
  this->Acelerometro = new Adxl345(module);
  this->Gyrometro = new Gyro(module);
  this->Magnetometro= new MAG(module);
}

/*
void IMU::Accel_Init()
{
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2D);  // Power register
  WIRE_SEND(0x08);  // Measurement mode
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x31);  // Data format register
  WIRE_SEND(0x08);  // Set to full resolution
  Wire.endTransmission();
  delay(5);
  
  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2C);  // Rate
  WIRE_SEND(0x09);  // Set to 50Hz, normal operation
  Wire.endTransmission();
  delay(5);
}*/

// Reads x, y and z accelerometer registers

void IMU::Read_Accel()
{
  this->Acelerometro->readFullAcel();
  accel[0] = this->Acelerometro->getAcelX();  // X axis (internal sensor y axis)
  accel[1] = this->Acelerometro->getAcelY();  // Y axis (internal sensor x axis)
  accel[2] = this->Acelerometro->getAcelZ();  // Z axis (internal sensor z axis)

}

void IMU::Read_Gyro(){
  this->Gyrometro->readFullGyro();
  gyro[0] = this->Gyrometro->getGyroX() * 1.05;  // X axis (internal sensor y axis)
  gyro[1] = this->Gyrometro->getGyroY() * 1.05;  // Y axis (internal sensor x axis)
  gyro[2] = this->Gyrometro->getGyroZ() * 1.05;  // Z axis (internal sensor z axis)

}
void IMU::Read_Magn(){
  this->Magnetometro->readfullState();
  magnetom[0]=this->Magnetometro->getMagX();
  magnetom[1]=this->Magnetometro->getMagY();
  magnetom[2]=this->Magnetometro->getMagZ();
}
/*
void IMU::Magn_Init()
{
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x02); 
  WIRE_SEND(0x00);  // Set continuous mode (default 10Hz)
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x00);
  WIRE_SEND(0b00011000);  // Set 50Hz
  Wire.endTransmission();
  delay(5);
}

void IMU::Read_Magn()
{
  short i = 0;
  byte buff[6];
 
  Wire.beginTransmission(MAGN_ADDRESS); 
  WIRE_SEND(0x03);  // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(MAGN_ADDRESS); 
  Wire.requestFrom(MAGN_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6)  // All bytes received?
  {
// 9DOF Razor IMU SEN-10125 using HMC5843 magnetometer
#if HW__VERSION_CODE == 10125
    // MSB byte first, then LSB; X, Y, Z
    magnetom[0] = -1 * (float)(short)(((((short) buff[2]) << 8) | ((short) buff[3])));  // X axis (internal sensor -y axis)
    magnetom[1] = -1 * (float)(short)(((((short) buff[0]) << 8) | ((short) buff[1])));  // Y axis (internal sensor -x axis)
    magnetom[2] = -1 * (float)(short)(((((short) buff[4]) << 8) | ((short) buff[5])));  // Z axis (internal sensor -z axis)
// 9DOF Razor IMU SEN-10736 using HMC5883L magnetometer
#elif HW__VERSION_CODE == 10736
    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
    magnetom[0] = -1 * (float)(short)(((((short) buff[4]) << 8) | ((short) buff[5])));  // X axis (internal sensor -y axis)
    magnetom[1] = -1 * (float)(short)(((((short) buff[0]) << 8) | ((short) buff[1])));  // Y axis (internal sensor -x axis)
    magnetom[2] = -1 * (float)(short)(((((short) buff[2]) << 8) | ((short) buff[3])));  // Z axis (internal sensor -z axis)
// 9DOF Sensor Stick SEN-10183 and SEN-10321 using HMC5843 magnetometer
#elif (HW__VERSION_CODE == 10183) || (HW__VERSION_CODE == 10321)
    // MSB byte first, then LSB; X, Y, Z
    magnetom[0] = (float)(short)((((short) buff[0]) << 8) | ((short) buff[1]));         // X axis (internal sensor x axis)
    magnetom[1] = -1 * (float)(short)(((((short) buff[2]) << 8) | ((short) buff[3])));  // Y axis (internal sensor -y axis)
    magnetom[2] = -1 * (float)(short)(((((short) buff[4]) << 8) | ((short) buff[5])));  // Z axis (internal sensor -z axis)
// 9DOF Sensor Stick SEN-10724 using HMC5883L magnetometer
#elif HW__VERSION_CODE == 10724
    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
    magnetom[0] = (float)(short)((((short) buff[0]) << 8) | ((short) buff[1]));         // X axis (internal sensor x axis)
    magnetom[1] = -1 * (float)(short)(((((short) buff[4]) << 8) | ((short) buff[5])));  // Y axis (internal sensor -y axis)
    magnetom[2] = -1 * (float)(short)(((((short) buff[2]) << 8) | ((short) buff[3])));  // Z axis (internal sensor -z axis)
#endif
  }
  else
  {
    num_magn_errors++;
    //if (output_errors) Serial.println("!ERR: reading magnetometer");
  }
}

void IMU::Gyro_Init()
{
  // Power up reset defaults
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x80);
  Wire.endTransmission();
  delay(5);
  
  // Select full-scale range of the gyro sensors
  // Set LP filter bandwidth to 42Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x16);
  WIRE_SEND(0x1B);  // DLPF_CFG = 3, FS_SEL = 3
  Wire.endTransmission();
  delay(5);
  
  // Set sample rato to 50Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x15);
  WIRE_SEND(0x0A);  //  SMPLRT_DIV = 10 (50Hz)
  Wire.endTransmission();
  delay(5);

  // Set clock to PLL with z gyro reference
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x00);
  Wire.endTransmission();
  delay(5);
}

// Reads x, y and z gyroscope registers
void IMU::Read_Gyro()
{
  short i = 0;
  byte buff[6];
  
  Wire.beginTransmission(GYRO_ADDRESS); 
  WIRE_SEND(0x1D);  // Sends address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.requestFrom(GYRO_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6)  // All bytes received?
  {
    gyro[0] = -1 * (float)(short)(((((short) buff[2]) << 8) | ((short) buff[3])));    // X axis (internal sensor -y axis)
    gyro[1] = -1 * (float)(short)(((((short) buff[0]) << 8) | ((short) buff[1])));    // Y axis (internal sensor -x axis)
    gyro[2] = -1 * (float)(short)(((((short) buff[4]) << 8) | ((short) buff[5])));    // Z axis (internal sensor -z axis)
  }
  else
  {
    num_gyro_errors++;
    //if (output_errors) Serial.println("!ERR: reading gyroscope");
  }
}*/

////////////////////////////////////////////////////////////////////////////////

void IMU::read_sensors() {

  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void IMU::reset_sensor_fusion() {
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  read_sensors();
  //timestamp = millis();
  
  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  float pitch_TMP = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));
  //
  if (std::isfinite(pitch_TMP))
    pitch = pitch_TMP;
  
  // GET ROLL
  // Compensate pitch of gravity vector 
  Vector_Cross_Product(temp1, accel, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  float roll_TMP = atan2(temp2[1], temp2[2]);
  //
  if (std::isfinite(roll_TMP))
    roll = roll_TMP;
  
  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;
  
  // Init rotation matrix
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

// Apply calibration to raw sensor readings
void IMU::compensate_sensor_errors() {
    // Compensate accelerometer error
    accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

    // Compensate magnetometer error
#if CALIBRATION__MAGN_USE_EXTENDED == true
    for (short i = 0; i < 3; i++)
      magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
    Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
#else
    magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
    magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
    magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
#endif

    // Compensate gyroscope error
    gyro[0] -= GYRO_AVERAGE_OFFSET_X;
    gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
    gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}

// Reset calibration session if reset_calibration_session_flag is set
void IMU::check_reset_calibration_session()
{
  // Raw sensor values have to be read already, but no error compensation applied

  // Reset this calibration session?
  if (!reset_calibration_session_flag) return;
  
  // Reset acc and mag calibration variables
  for (short i = 0; i < 3; i++) {
    accel_min[i] = accel_max[i] = accel[i];
    magnetom_min[i] = magnetom_max[i] = magnetom[i];
  }

  // Reset gyro calibration variables
  gyro_num_samples = 0;  // Reset gyro calibration averaging
  gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;
  
  reset_calibration_session_flag = false;
}

void IMU::turn_output_stream_on()
{
  output_stream_on = true;
  //digitalWrite(STATUS_LED_PIN, HIGH);
}

void IMU::turn_output_stream_off()
{
  output_stream_on = false;
  //digitalWrite(STATUS_LED_PIN, LOW);
}

// Blocks until another byte is available on serial port

/*
char IMU::readChar()
{
  while (Serial.available() < 1) { } // Block
  return Serial.read();
}*/
 
void IMU::setup()
{
  //Wire.setModule(module);

  // Init status LED
  //pinMode (STATUS_LED_PIN, OUTPUT);
  //digitalWrite(STATUS_LED_PIN, LOW);

  // Init sensors
  //delay(50);  // Give sensors enough time to start
  I2C_Init();
  //Accel_Init();
  //Magn_Init();
  //Gyro_Init();
  
  // Read sensors, init DCM algorithm
  usleep(20000);  // Give sensors enough time to collect data
  reset_sensor_fusion();

  // Init output
#if (OUTPUT__HAS_RN_BLUETOOTH == true) || (OUTPUT__STARTUP_STREAM_ON == false)
  turn_output_stream_off();
#else
  turn_output_stream_on();
#endif
}



// Main loop
void IMU::loop()
{ 
  // Time to read the sensors again?

  //if((millis() - timestamp) >= OUTPUT__DATA_INTERVAL)
  //{
  //
  //  timestamp_old = timestamp;
  //  timestamp = millis();
  //  if (timestamp > timestamp_old)
  //    G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
  //  else G_Dt = 0;
    
    // Update sensor readings
    //Wire.setModule(module);
    G_Dt = .020;
    read_sensors();

    // Apply sensor calibration
    compensate_sensor_errors();
    
    // Run DCM algorithm
    Compass_Heading(); // Calculate magnetic heading
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
  //}
}