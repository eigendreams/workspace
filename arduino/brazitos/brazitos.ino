#include "ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Empty.h"
#include "Servo.h"
#include "Talon.h"
#include "Encoder.h"

////////////////////////////////////////////////////////////////////////////////
#include <i2cmaster.h>
#include "MLX90620_registers.h"

#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

std_msgs::MultiArrayDimension dimws;
std_msgs::Int16MultiArray ws;
ros::Publisher ir_pub("ws", &ws);

int refreshRate = 16; //Set this value to your desired refresh frequency

int conta=0;
//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
int irData[64]; //Contains the raw IR data from the sensor
//float temperatures[64]; //Contains the calculated temperatures of each pixel in the array
//float Tambient; //Tracks the changing ambient temperature of the sensor
//byte eepromData[256]; //Contains the full EEPROM reading from the MLX (Slave 0x50)

//These are constants calculated from the calibration data stored in EEPROM
//See varInitialize and section 7.3 for more information
//int v_th, a_cp, b_cp, tgc, b_i_scale;
//float k_t1, k_t2, emissivity;
//int a_ij[64], b_ij[64];

//These values are calculated using equation 7.3.3.2
//They are constants and can be calculated using the MLX90620_alphaCalculator sketch
/*float alpha_ij[64] = {
  1.67684E-8, 1.85146E-8, 1.87474E-8, 1.67684E-8, 1.87474E-8, 2.04936E-8, 2.04936E-8, 1.79325E-8, 
  2.00862E-8, 2.20653E-8, 2.16578E-8, 1.93295E-8, 2.10757E-8, 2.32294E-8, 2.28220E-8, 2.04936E-8, 
  2.18324E-8, 2.43936E-8, 2.41607E-8, 2.16578E-8, 2.28220E-8, 2.49756E-8, 2.49756E-8, 2.26473E-8, 
  2.32294E-8, 2.53249E-8, 2.57323E-8, 2.34040E-8, 2.32294E-8, 2.61398E-8, 2.59070E-8, 2.38115E-8, 
  2.32294E-8, 2.59070E-8, 2.61398E-8, 2.39861E-8, 2.29966E-8, 2.57323E-8, 2.61398E-8, 2.38115E-8, 
  2.28220E-8, 2.57323E-8, 2.57323E-8, 2.38115E-8, 2.26473E-8, 2.53249E-8, 2.53249E-8, 2.34040E-8, 
  2.12503E-8, 2.43936E-8, 2.51503E-8, 2.29966E-8, 2.00862E-8, 2.28220E-8, 2.32294E-8, 2.20070E-8, 
  1.81653E-8, 2.08429E-8, 2.22399E-8, 2.04936E-8, 1.61863E-8, 1.95041E-8, 1.99116E-8, 1.85146E-8, 
};*/

byte loopCount = 0; //Used in main loop
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

/*//From the 256 bytes of EEPROM data, initialize 
void varInitialization(byte calibration_data[])
{
  v_th = 256 * calibration_data[VTH_H] + calibration_data[VTH_L];
  k_t1 = (256 * calibration_data[KT1_H] + calibration_data[KT1_L]) / 1024.0; //2^10 = 1024
  k_t2 = (256 * calibration_data[KT2_H] + calibration_data[KT2_L]) / 1048576.0; //2^20 = 1,048,576
  emissivity = ((unsigned int)256 * calibration_data[CAL_EMIS_H] + calibration_data[CAL_EMIS_L]) / 32768.0;
  
  a_cp = calibration_data[CAL_ACP];
  if(a_cp > 127) a_cp -= 256; //These values are stored as 2's compliment. This coverts it if necessary.

  b_cp = calibration_data[CAL_BCP];
  if(b_cp > 127) b_cp -= 256;

  tgc = calibration_data[CAL_TGC];
  if(tgc > 127) tgc -= 256;

  b_i_scale = calibration_data[CAL_BI_SCALE];

  for(int i = 0 ; i < 64 ; i++)
  {
    //Read the individual pixel offsets
    a_ij[i] = calibration_data[i]; 
    if(a_ij[i] > 127) a_ij[i] -= 256; //These values are stored as 2's compliment. This coverts it if necessary.

    //Read the individual pixel offset slope coefficients
    b_ij[i] = calibration_data[0x40 + i]; //Bi(i,j) begins 64 bytes into EEPROM at 0x40
    if(b_ij[i] > 127) b_ij[i] -= 256;
  }
  
}*/

//Receives the refresh rate for sensor scanning
//Sets the two byte configuration registers
//This function overwrites what is currently in the configuration registers
//The MLX doesn't seem to mind this (flags are read only)
void setConfiguration(int irRefreshRateHZ)
{
  byte Hz_LSB;

  switch(irRefreshRateHZ)
  {
  case 0:
    Hz_LSB = 0b00001111;
    break;
  case 1:
    Hz_LSB = 0b00001110;
    break;
  case 2:
    Hz_LSB = 0b00001101;
    break;
  case 4:
    Hz_LSB = 0b00001100;
    break;
  case 8:
    Hz_LSB = 0b00001011;
    break;
  case 16:
    Hz_LSB = 0b00001010;
    break;
  case 32:
    Hz_LSB = 0b00001001;
    break;
  default:
    Hz_LSB = 0b00001110;
  }

  byte defaultConfig_H = 0b01110100; // x111.01xx, Assumes NA = 0, ADC low reference enabled, Ta Refresh rate of 2Hz

  i2c_start_wait(MLX90620_WRITE);
  i2c_write(0x03); //Command = configuration value
  i2c_write((byte)Hz_LSB - 0x55);
  i2c_write(Hz_LSB);
  i2c_write(defaultConfig_H - 0x55); //Assumes NA = 0, ADC low reference enabled, Ta Refresh rate of 2Hz
  i2c_write(defaultConfig_H);
  i2c_stop();
}

/*//Read the 256 bytes from the MLX EEPROM and setup the various constants (*lots* of math)
//Note: The EEPROM on the MLX has a different I2C address from the MLX. I've never seen this before.
void read_EEPROM_MLX90620()
{
  i2c_start_wait(MLX90620_EEPROM_WRITE);
  i2c_write(0x00); //EEPROM info starts at location 0x00
  i2c_rep_start(MLX90620_EEPROM_READ);

  //Read all 256 bytes from the sensor's EEPROM
  for(int i = 0 ; i <= 255 ; i++)
    eepromData[i] = i2c_readAck();

  i2c_stop(); //We're done talking

  //varInitialization(eepromData); //Calculate a bunch of constants from the EEPROM data

  writeTrimmingValue(eepromData[OSC_TRIM_VALUE]);
}*/

//Given a 8-bit number from EEPROM (Slave address 0x50), write value to MLX sensor (Slave address 0x60)
void writeTrimmingValue(byte val)
{
  i2c_start_wait(MLX90620_WRITE); //Write to the sensor
  i2c_write(0x04); //Command = write oscillator trimming value
  i2c_write((byte)val - 0xAA);
  i2c_write(val);
  i2c_write(0x56); //Always 0x56
  i2c_write(0x00); //Always 0x00
  i2c_stop();
}

/*//Gets the latest PTAT (package temperature ambient) reading from the MLX
//Then calculates a new Tambient
//Many of these values (k_t1, v_th, etc) come from varInitialization and EEPROM reading
//This has been tested to match example 7.3.2
void calculate_TA(void)
{
  unsigned int ptat = readPTAT_MLX90620();

  Tambient = (-k_t1 + sqrt(square(k_t1) - (4 * k_t2 * (v_th - (float)ptat)))) / (2*k_t2) + 25; //it's much more simple now, isn't it? :)
}*/

//Reads the PTAT data from the MLX
//Returns an unsigned int containing the PTAT
unsigned int readPTAT_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE);
  i2c_write(CMD_READ_REGISTER); //Command = read PTAT
  i2c_write(0x90); //Start address is 0x90
  i2c_write(0x00); //Address step is 0
  i2c_write(0x01); //Number of reads is 1
  i2c_rep_start(MLX90620_READ);

  byte ptatLow = i2c_readAck(); //Grab the lower and higher PTAT bytes
  byte ptatHigh = i2c_readAck();

  i2c_stop();
  
  return( (unsigned int)(ptatHigh << 8) | ptatLow); //Combine bytes and return
}

/*//Calculate the temperatures seen for each pixel
//Relies on the raw irData array
//Returns an 64-int array called temperatures
void calculate_TO()
{
  float v_ir_off_comp;
  float v_ir_tgc_comp;
  float v_ir_comp;

  //Calculate the offset compensation for the one compensation pixel
  //This is a constant in the TO calculation, so calculate it here.
  int cpix = readCPIX_MLX90620(); //Go get the raw data of the compensation pixel
  float v_cp_off_comp = (float)cpix - (a_cp + (b_cp/pow(2, b_i_scale)) * (Tambient - 25)); 

  for (int i = 0 ; i < 64 ; i++)
  {
    v_ir_off_comp = irData[i] - (a_ij[i] + (float)(b_ij[i]/pow(2, b_i_scale)) * (Tambient - 25)); //#1: Calculate Offset Compensation 

    v_ir_tgc_comp = v_ir_off_comp - ( ((float)tgc/32) * v_cp_off_comp); //#2: Calculate Thermal Gradien Compensation (TGC)

    v_ir_comp = v_ir_tgc_comp / emissivity; //#3: Calculate Emissivity Compensation

    //emperatures[i] = sqrt( sqrt( (v_ir_comp/alpha_ij[i]) + pow(Tambient + 273.15, 4) )) - 273.15;
  }
}*/

//Reads 64 bytes of pixel data from the MLX
//Loads the data into the irData array
void readIR_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE);
  i2c_write(CMD_READ_REGISTER); //Command = read a register
  i2c_write(0x00); //Start address = 0x00
  i2c_write(0x01); //Address step = 1
  i2c_write(0x40); //Number of reads is 64
  i2c_rep_start(MLX90620_READ);

  for(int i = 0 ; i < 64 ; i++)
  {
    byte pixelDataLow = i2c_readAck();
    byte pixelDataHigh = i2c_readAck();
    irData[i] = (int)(pixelDataHigh << 8) | pixelDataLow;
  }

  i2c_stop();
}

//Read the compensation pixel 16 bit data
int readCPIX_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE);
  i2c_write(CMD_READ_REGISTER); //Command = read register
  i2c_write(0x91);
  i2c_write(0x00);
  i2c_write(0x01);
  i2c_rep_start(MLX90620_READ);

  byte cpixLow = i2c_readAck(); //Grab the two bytes
  byte cpixHigh = i2c_readAck();
  i2c_stop();

  return ( (int)(cpixHigh << 8) | cpixLow);
}

//Reads the current configuration register (2 bytes) from the MLX
//Returns two bytes
unsigned int readConfig_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE); //The MLX configuration is in the MLX, not EEPROM
  i2c_write(CMD_READ_REGISTER); //Command = read configuration register
  i2c_write(0x92); //Start address
  i2c_write(0x00); //Address step of zero
  i2c_write(0x01); //Number of reads is 1

    i2c_rep_start(MLX90620_READ);

  byte configLow = i2c_readAck(); //Grab the two bytes
  byte configHigh = i2c_readAck();

  i2c_stop();

  return( (unsigned int)(configHigh << 8) | configLow); //Combine the configuration bytes and return as one unsigned int
}

//Poll the MLX for its current status
//Returns true if the POR/Brown out bit is set
boolean checkConfig_MLX90620()
{
  if ( (readConfig_MLX90620() & (unsigned int)1<<POR_TEST) == 0)
    return true;
  else
    return false;
}

/*//Prints the temperatures in a way that's more easily viewable in the terminal window
void prettyPrintTemperatures()
{
  Serial.println();
  for(int i = 0 ; i < 64 ; i++)
  {
    if(i % 16 == 0) Serial.println();
    Serial.print(temperatures[i]);
    //Serial.print(irData[i]);
    Serial.print(", ");
  }
}*/

/*//Prints the temperatures in a way that's more easily parsed by a Processing app
//Each line starts with '$' and ends with '*'
void rawPrintTemperatures()
{
  Serial.print("$");
  for(int i = 0 ; i < 64 ; i++)
  {
    Serial.print(temperatures[i]);
    if(i!=63){
      Serial.print(","); //Don't print comma on last temperature
    }
  }
  Serial.println("*");
}*/

/*//Given a Celsius float, converts to Fahrenheit
float convertToFahrenheit (float Tc)
{
  float Tf = (9/5) * Tc + 32;

  return(Tf);
}*/
////////////////////////////////////////////////////////////////////////////////

ros::NodeHandle nh;

// umbral, max
TalonClass MTFR(2, 70);
TalonClass MTFL(2, 70);
TalonClass MTBR(2, 70);
TalonClass MTBL(2, 70);

// pin, min, max, max_change, map
EncoderClass ENCFR(A0, 0, 1023, 255, 100);
EncoderClass ENCFL(A1, 0, 1023, 255, 100);
EncoderClass ENCBR(A2, 0, 1023, 255, 100);
EncoderClass ENCBL(A3, 0, 1023, 255, 100);

// The incoming 6 DOF int16 information from ROS
int fr_out = 0;
int fl_out = 0;
int br_out = 0;
int bl_out = 0;

// Encoder lectures
int fr_lec;
int fl_lec;
int br_lec;
int bl_lec;

unsigned long milisLast = 0;
unsigned long milisLastMsg = 0;
bool timedOut = false;

void fr_out_cb(const std_msgs::Int16& dmsg) {fr_out = dmsg.data;}
void fl_out_cb(const std_msgs::Int16& dmsg) {fl_out = dmsg.data;}
void br_out_cb(const std_msgs::Int16& dmsg) {br_out = dmsg.data;}
void bl_out_cb(const std_msgs::Int16& dmsg) {bl_out = dmsg.data;}
void alive_cb( const std_msgs::Int16& dmsg) {
  milisLastMsg = millis();
  timedOut = false;
}
ros::Subscriber<std_msgs::Int16> fr_out_sub("fr_out", fr_out_cb);
ros::Subscriber<std_msgs::Int16> fl_out_sub("fl_out", fl_out_cb);
ros::Subscriber<std_msgs::Int16> br_out_sub("br_out", br_out_cb);
ros::Subscriber<std_msgs::Int16> bl_out_sub("bl_out", bl_out_cb);
ros::Subscriber<std_msgs::Int16> alive_sub("alive", alive_cb);

std_msgs::Int16 fr_lec_msg;
std_msgs::Int16 fl_lec_msg;
std_msgs::Int16 br_lec_msg;
std_msgs::Int16 bl_lec_msg;
ros::Publisher fr_lec_pub("fr_lec", &fr_lec_msg);
ros::Publisher fl_lec_pub("fl_lec", &fl_lec_msg);
ros::Publisher br_lec_pub("br_lec", &br_lec_msg);
ros::Publisher bl_lec_pub("bl_lec", &bl_lec_msg);

void setup() {
  MTFR.attach(3);
  MTFL.attach(5);
  MTBR.attach(6);
  MTBL.attach(9);

  nh.initNode();

  nh.subscribe(fr_out_sub);
  nh.subscribe(fl_out_sub);
  nh.subscribe(br_out_sub);
  nh.subscribe(bl_out_sub);
  nh.subscribe(alive_sub);
  nh.advertise(fr_lec_pub);
  nh.advertise(fl_lec_pub);
  nh.advertise(br_lec_pub);
  nh.advertise(bl_lec_pub);

  //////////////////////////////////////////////////////////////////////////////
  ws.layout.dim_length = 1;
  ws.data_length = 64;
  ws.layout.dim = &dimws;
  ws.layout.dim[0].label = "ws";
  ws.layout.dim[0].size = 64;
  ws.layout.dim[0].stride = 64;
  ws.layout.data_offset = 0;
  ws.data = &irData[0];

  i2c_init(); //Init the I2C pins
  //PORTC = (1 << PORTC4) | (1 << PORTC5); //Enable pull-ups
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  delay(5); //Init procedure calls for a 5ms delay after power-on
  writeTrimmingValue(100);
  setConfiguration(refreshRate); //Configure the MLX sensor with the user's choice of refresh rate
  //calculate_TA(); //Calculate the current Tambient

  nh.advertise(ir_pub);
  //////////////////////////////////////////////////////////////////////////////
}

void loop() {
  nh.spinOnce();

  unsigned long milisNow = millis();

  if (milisNow - milisLast >= 100) {

    // Check for timeOut condition, if yes set desired speeds to 0 and raise the timedOut flag
    // to set mode as PWM until next message is received (default timeOut as used in ROS, 5000 ms)
    if (milisNow - milisLastMsg >= 3500) {
      fr_out = 0;
      fl_out = 0;
      br_out = 0;
      bl_out = 0;
      timedOut = true;
    }

    // Obten los valores absolutos de los encoders
    fr_lec = ENCFR.read();
    fl_lec = ENCFL.read();
    br_lec = ENCBR.read();
    bl_lec = ENCBL.read();

    // Y darle salida a cada motor en cada DOF
    MTFR.write(fr_out);
    MTFL.write(fl_out);
    MTBR.write(br_out);
    MTBL.write(bl_out);
    
    fr_lec_msg.data = fr_lec;
    fl_lec_msg.data = fl_lec;
    br_lec_msg.data = br_lec;
    bl_lec_msg.data = bl_lec;

    fr_lec_pub.publish(&fr_lec_msg);
    fl_lec_pub.publish(&fl_lec_msg);
    br_lec_pub.publish(&br_lec_msg);
    bl_lec_pub.publish(&bl_lec_msg);

    milisLast = milisNow;
  }

  //////////////////////////////////////////////////////////////////////////////
  if(loopCount++ == 16) //Tambient changes more slowly than the pixel readings. Update TA only every 16 loops.
  { 
    //calculate_TA(); //Calculate the new Tambient
    if(checkConfig_MLX90620()) //Every 16 readings check that the POR flag is not set
    {
      setConfiguration(refreshRate); //Re-write the configuration bytes to the MLX
    }
    loopCount = 0; //Reset count
  }
  readIR_MLX90620(); //Get the 64 bytes of raw pixel data into the irData array
  //calculate_TO(); //Run all the large calculations to get the temperature data for each pixel
  conta++;
  if(conta>20){
    conta=0;
    ir_pub.publish(&ws);
  }
  //rawPrintTemperatures(); //Print the entire array so it can more easily be read by Processing app
  //////////////////////////////////////////////////////////////////////////////
}
