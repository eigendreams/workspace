#include "Arduino.h"
#include "Servo.h"
#include "ros.h"
#include "std_msgs/Int16.h"
#include "SimpleServo.h"
#include "SimpleDriver.h"

ros::NodeHandle nh;

// Pins 3, 5 for DC Motor control (first DOF) and pins 4, 6 for DC Motor Control (fourth DOF)
// SimpleDriverClass(pin_en, pin_dir, umbral, val) // sense == sign(val), max_angle == abs(val)
SimpleDriverClass motor1(5, 4, 3, 2, 100);
SimpleDriverClass motor2(6, 7, 8, 2, 100);

// Pin 9 for standard servo control (fifth DOF)
// SimpleServoClass(pin, val) // sense == sign(val), max_angle == abs(val) (from -90 to +90)
Servo servoobj1;
Servo servoobj2;
SimpleServoClass servo1(&servoobj1, 90);
SimpleServoClass servo2(&servoobj2, 90);

unsigned long milisLast = 0;
unsigned long timeLastMsg = 0;	// used to watch for timeOut
bool timedOut = false;

int motor1des = 0;
int motor2des = 0;
int servo1des = 0;
int servo2des = 0;

// All the subs handlers functions, update is asynchronous (done in the loop), keep names short
void motor1_h(const std_msgs::Int16& d_msg) {	motor1des = d_msg.data;	timedOut = false;	timeLastMsg = millis();	}
void motor2_h(const std_msgs::Int16& d_msg) {	motor2des = d_msg.data;	timedOut = false;	timeLastMsg = millis();	}
void servo1_h(const std_msgs::Int16& d_msg) {	servo1des = d_msg.data;	timedOut = false;	timeLastMsg = millis();	}
void servo2_h(const std_msgs::Int16& d_msg) {	servo2des = d_msg.data;	timedOut = false;	timeLastMsg = millis();	}
ros::Subscriber<std_msgs::Int16> motor_sub1("motor1", motor1_h);
ros::Subscriber<std_msgs::Int16> motor_sub2("motor2", motor2_h);
ros::Subscriber<std_msgs::Int16> servo_sub1("servo1", servo1_h);
ros::Subscriber<std_msgs::Int16> servo_sub2("servo2", servo2_h);

// All the pubs msgs, keep names short, alive is seconds from start, offset must be managed in pc
std_msgs::Int16 alive;
ros::Publisher alive_pub("alive", &alive);

void setup() {
	motor1.begin();
	motor2.begin();

	pinMode(3, OUTPUT);
	pinMode(4, OUTPUT);
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(7, OUTPUT);
	pinMode(8, OUTPUT);
	
	servo1.attach(9);
	servo2.attach(10);

	nh.initNode();

	nh.advertise(alive_pub);
	nh.subscribe(motor_sub1);
	nh.subscribe(motor_sub2);
	nh.subscribe(servo_sub1);
	nh.subscribe(servo_sub2);
}

void loop() {
	nh.spinOnce();

	unsigned long milisNow = millis();

	// 20 Hz / 50 ms operation is best, it seems
	if (milisNow - milisLast >= 200) {

		// Check for timeOut condition, if yes set desired speeds to 0 and raise the timedOut flag
		// to set mode as PWM until next message is received (default timeOut as used in ROS, 5000 ms)
		if (milisNow - timeLastMsg >= 5000) {
			motor1des = 0;
			motor2des = 0;
			servo1des = 0;
			servo2des = 0;
			timedOut = true;
		}

		motor1.write(motor1des);
		motor2.write(motor2des);
		servo1.write(servo1des);
		servo2.write(servo2des);	

		alive.data = millis() / 1000;
		alive_pub.publish(&alive);

		milisLast = milisNow;
	}
}
