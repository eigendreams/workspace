#include "ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Empty.h"
#include "Servo.h"
#include "Talon.h"
#include "Encoder.h"

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
void alive_cb(const std_msgs::Empty& dmsg) {
  milisLastMsg = millis();
  timedOut = false;
}
ros::Subscriber<std_msgs::Int16> fr_out_sub("fr_out", fr_out_cb);
ros::Subscriber<std_msgs::Int16> fl_out_sub("fl_out", fl_out_cb);
ros::Subscriber<std_msgs::Int16> br_out_sub("br_out", br_out_cb);
ros::Subscriber<std_msgs::Int16> bl_out_sub("bl_out", bl_out_cb);
ros::Subscriber<std_msgs::Empty> alive_sub("alive", alive_cb);

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
}
