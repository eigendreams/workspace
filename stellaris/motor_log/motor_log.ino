#include "ros.h"
#include "std_msgs/Int16.h"
#include "Servo.h"
#include "AS5043.h"
#include "Encoder.h"

ros::NodeHandle nh;

unsigned long millisLast = 0;
unsigned long millisLastMsg = 0;
bool timedOut = false;
Servo servo_13;
Servo servo_14;
AS5043Class AS5043obj(15, 25, 26);
EncoderClass gear1(&AS5043obj, 34, 485, 1004, -100);
EncoderClass gear2(&AS5043obj, 33, 485, 1004, -100);
volatile int s13_out = 0;  // m1
volatile int s14_out = 0;  // m2
//
void s13_out_cb(const std_msgs::Int16& dmsg) {  
  s13_out = dmsg.data;  
}
ros::Subscriber<std_msgs::Int16> s13_out_sub("m1", s13_out_cb);
void s14_out_cb(const std_msgs::Int16& dmsg) {  
  s14_out = dmsg.data;  
}
ros::Subscriber<std_msgs::Int16> s14_out_sub("m2", s14_out_cb);
//
void alive_cb( const std_msgs::Int16& dmsg) {
  if (dmsg.data){
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    millisLastMsg = millis();
    timedOut = false;
  }
} 
ros::Subscriber<std_msgs::Int16> alive_sub("al", alive_cb);

int gear1_lec;
int gear2_lec;
std_msgs::Int16 gear1_lec_msg;
std_msgs::Int16 gear2_lec_msg;
ros::Publisher gear1_lec_pub("e1", &gear1_lec_msg);
ros::Publisher gear2_lec_pub("e2", &gear2_lec_msg);

void setup() {

  nh.initNode();

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);

  pinMode(15, OUTPUT);
  pinMode(25, INPUT);
  pinMode(26, OUTPUT);
  pinMode(34, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(32, OUTPUT);

  servo_13.attach(13);
  servo_14.attach(14);

  nh.subscribe(s13_out_sub);
  nh.subscribe(s14_out_sub);
  //nh.subscribe(s15_out_sub);
  nh.subscribe(alive_sub);

  nh.advertise(gear1_lec_pub);
  nh.advertise(gear2_lec_pub);

  delay(50);

  AS5043obj.begin();
}


void loop() {

  nh.spinOnce();

  unsigned long millisNow = millis();
  if (millisNow - millisLast >= 10) {
    millisLast = millisNow;

    if (millisNow - millisLastMsg >= 3000) {
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
      s13_out = 0;  // m1
      timedOut = true;
    }

    gear1_lec = gear1.read();
    delayMicroseconds(222);
    gear2_lec = gear2.read();
    //
    if (gear1.isValid() != 1) {
      gear1.restartComm();
      gear1_lec = gear1.read();
    }
    //
    if (gear2.isValid() != 1) {
      gear2.restartComm();
      gear2_lec = gear2.read();
    }
    //gearv_lec = gearv.read();

    gear1_lec_msg.data = gear1_lec;
    gear2_lec_msg.data = gear2_lec;
    //gearv_lec_msg.data = gearv_lec;

    gear1_lec_pub.publish(&gear1_lec_msg);
    gear2_lec_pub.publish(&gear2_lec_msg);
    //gearv_lec_pub.publish(&gearv_lec_msg);

    //servo_11.writeMicroseconds(100 * s11_out + 1000); // v1
    //servo_12.writeMicroseconds(100 * s12_out + 1000); // v2
    servo_13.writeMicroseconds( 50 * s13_out + 1500); // m1
    servo_14.writeMicroseconds( 50 * s14_out + 1500); // m2
    //servo_15.writeMicroseconds( 50 * s15_out + 1500); // mv
  }
}

