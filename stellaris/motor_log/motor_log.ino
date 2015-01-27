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
AS5043Class AS5043obj(15, 25, 26);
EncoderClass gear1(&AS5043obj, 34, 485, 1004, -100);
volatile int s13_out = 0;  // m1

void s13_out_cb(const std_msgs::Int16& dmsg) {	
  s13_out = dmsg.data;	
}
ros::Subscriber<std_msgs::Int16> s13_out_sub("m1", s13_out_cb);

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
std_msgs::Int16 gear1_lec_msg;
ros::Publisher gear1_lec_pub("e1", &gear1_lec_msg);

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

  nh.subscribe(s13_out_sub);
  nh.subscribe(alive_sub);

  nh.advertise(gear1_lec_pub);

  delay(50);

  AS5043obj.begin();
}


void loop() {

  nh.spinOnce();

  unsigned long millisNow = millis();
  if (millisNow - millisLast >= 100) {
    millisLast = millisNow;

    if (millisNow - millisLastMsg >= 3000) {
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
      s13_out = 0;  // m1
      timedOut = true;
    }

    gear1_lec = gear1.read();
    gear1_lec_msg.data = gear1_lec;
    gear1_lec_pub.publish(&gear1_lec_msg);

    servo_13.writeMicroseconds( 50 * s13_out + 1500); // m1
  }
}

