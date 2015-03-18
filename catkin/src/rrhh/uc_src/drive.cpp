// The code is edited and compiled in Eclipse, using the Arduino Core library and AVR compiler
//
// The following code turns a "switching" relay of the RRHH on and off, and replicates the desired PWM values in the
// corresponding pins. Two SaberTooths (25 A x 2 channels) are used as motor controllers, using the analog input mode.
// A single stage filter is used to transform the PWM outputs to continuous voltage values.
// A motor speed of zero maps to an analog output of 2.5 V to the SaberTooth
//
// To reduce buffer size, ros.h has been modified to:
//
//	...
//  #elif defined(__AVR_ATmega328P__)
//  	typedef NodeHandle_<ArduinoHardware, 10, 10, 100, 100> NodeHandle;
//  ...

#include <Arduino.h>

#include <ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
// ********************************************************************************************************************
int motor1Pin = 5;
int motor2Pin = 6;
int motor3Pin = 9;
int motor4Pin = 10;
// ********************************************************************************************************************
void turn_off() {
	analogWrite(motor1Pin, 127);
	analogWrite(motor2Pin, 127);
	analogWrite(motor3Pin, 127);
	analogWrite(motor4Pin, 127);
}
// ********************************************************************************************************************
ros::NodeHandle nh;

bool mode_control_val = true;
bool mode_relay_val = false;

int input_vdes_1 = 0;
int input_vdes_2 = 0;
int input_vdes_3 = 0;
int input_vdes_4 = 0;

void tf_vdes(const std_msgs::Int16MultiArray& array) {
	if (mode_relay_val) {
		// The direction selection is done in the program loop, to save time. Anyway, this
		// callback should not be called more than 10 times per second
		input_vdes_1 = array.data[0];
		input_vdes_2 = array.data[1];
		input_vdes_3 = array.data[2];
		input_vdes_4 = array.data[3];
	} else {
		// Else the desired values are "zeroed", just in case
		input_vdes_1 = 127;
		input_vdes_2 = 127;
		input_vdes_3 = 127;
		input_vdes_4 = 127;
	}
}

void mode_control(const std_msgs::Bool& data) {
	// Specifies the control mode, if true, the vdes values are taken as PWM outputs to send
	// directly to the Sabertooth, otherwise they are taken as velocities
	// NOT USED/IMPLEMENTED YET
	mode_control_val = data.data;
}

void mode_relay(const std_msgs::Bool& data) {
	// Shut off should be fast, so it's done here. If done in the loop, the delay could be as big
	// as 200 ms. A True is needed to turn on the relay and thus the motors. Since the analogWrite
	// call for actually "moving" the motors is in loop(), there will be a small delay when turning on
	mode_relay_val = data.data;
	if (mode_relay_val)
		digitalWrite(7, HIGH);
	else {
		digitalWrite(7, LOW);
		turn_off();
	}
}

ros::Subscriber<std_msgs::Int16MultiArray> tf_vdes_sub("tf_vdes", tf_vdes);
ros::Subscriber<std_msgs::Bool> mode_control_sub("mc", mode_control);
ros::Subscriber<std_msgs::Bool> mode_relay_sub("mr", mode_relay);

void start_ros() {
	nh.initNode();
	nh.subscribe(tf_vdes_sub);
	nh.subscribe(mode_control_sub);
	nh.subscribe(mode_relay_sub);
}
// ********************************************************************************************************************
void setup() {
	start_ros();

	// Turn off the relay at the start
	digitalWrite(7, LOW);
	pinMode(7, OUTPUT);
}

void loop() {
	nh.spinOnce();

	if (mode_relay_val == false)
		turn_off();
	else {
		analogWrite(motor1Pin, input_vdes_1);
		analogWrite(motor2Pin, input_vdes_2);
		analogWrite(motor3Pin, input_vdes_3);
		analogWrite(motor4Pin, input_vdes_4);
	}
}
