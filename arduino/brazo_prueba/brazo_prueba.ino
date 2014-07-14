#define USE_USBCON
#include "ros.h"
#include "std_msgs/Int16.h"

#include "Servo.h"
#include "DynamixelSerial.h"
#include "SimpleDynamixel.h"
#include "SimpleDriver.h"
#include "Talon.h"
#include "SimpleServo.h"
#include "AS5043.h"
#include "Encoder.h"

ros::NodeHandle nh;
Servo munnieca_servo;
EncoderClass ENCMUNNIECA(A0, 0, 1023, 100, 100);
int wrist_out= 0;
int wrist_lec;
unsigned long milisLast = 0;
unsigned long milisLastMsg = 0;
bool timedOut = false;
void wrist_out_cb(const std_msgs::Int16& dmsg) {wrist_out = dmsg.data;}
void alive_cb(const std_msgs::Int16& dmsg) {
	if (dmsg.data) {
  	milisLastMsg = millis();
  	timedOut = false;
	}
}
ros::Subscriber<std_msgs::Int16> wrist_out_sub("wrist_out", wrist_out_cb);
ros::Subscriber<std_msgs::Int16> alive_sub("alive", alive_cb);
std_msgs::Int16 wrist_lec_msg;
ros::Publisher wrist_lec_pub("wrist_lec", &wrist_lec_msg);

void setup() {
	munnieca_servo.attach(9);
	pinMode(A0, INPUT);
	nh.initNode();
	nh.subscribe(wrist_out_sub);
	nh.subscribe(alive_sub);
	nh.advertise(wrist_lec_pub);
}

void loop() {
	nh.spinOnce();
	unsigned long milisNow = millis();
	munnieca_servo.write(wrist_out * 5 + 1500);
	// 10 Hz operation
	if (milisNow - milisLast >= 100) {
		milisLast = milisNow;
	    if (milisNow - milisLastMsg >= 3500) {
	      wrist_out = 0;		// CHECK THIS ONE!!!
	      timedOut = true;
	    }
	    wrist_lec = ENCMUNNIECA.read();
		munnieca_servo.write(wrist_out * 5 + 1500);
		wrist_lec_msg.data = wrist_lec;
		wrist_lec_pub.publish(&wrist_lec_msg); delay(1);
	}
}