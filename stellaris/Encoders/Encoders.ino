#include "ros.h"
#include "std_msgs/Int16.h"
#include "Servo.h"
#include "AS5043.h"
#include "Encoder.h"

ros::NodeHandle nh;

// CLK, DO, PROG,	CSN
//AS5043Class AS5043obj(PD_2, PD_0, PD_1);
//EncoderClass ENCBASE(&AS5043obj, PD_3, 485, 1004, -100);
// CLK, DO, PROG
AS5043Class AS5043obj(23, 25, 26);
// CSN
EncoderClass gear1(&AS5043obj, 34, 485, 1004, -100);

// The incoming 6 DOF int16 information from ROS
int base_out = 0;
int base_lec;

unsigned long milisLast = 0;
unsigned long milisLastMsg = 0;
bool timedOut = false;

std_msgs::Int16 base_lec_msg;
ros::Publisher base_lec_pub("basebase", &base_lec_msg);

void setup() {
	// Select encoders as digital
	// pinMode(PD_3, OUTPUT);
	nh.initNode();
	nh.advertise(base_lec_pub);

	pinMode(23, OUTPUT);
	pinMode(25, INPUT);
	pinMode(26, OUTPUT);
	pinMode(34, OUTPUT);

	AS5043obj.begin();
}

void loop() {
	nh.spinOnce();

	unsigned long milisNow = millis();

	// 20 Hz operation
	if (milisNow - milisLast >= 100) {

		milisLast = milisNow;

	    // Obten los valores absolutos de los encoders
	    // base_lec = ;
	    //AS5043obj.setPinCSn(34);
	    base_lec = gear1.read();
		base_lec_msg.data = base_lec;
		base_lec_pub.publish(&base_lec_msg); delay(1);
	}
}