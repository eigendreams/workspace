#include "Servo.h"
#include "AS5043.h"
#include "Encoder.h"

unsigned long milisLast = 0;
unsigned long milisLastMsg = 0;
unsigned long millisTock = 0;
unsigned long timescont = 0;
bool timedOut = false;

Servo servo_13;
Servo servo_14;

AS5043Class AS5043obj(23, 25, 26);
EncoderClass gear1(&AS5043obj, 34, 485, 1004, -100);
EncoderClass gear2(&AS5043obj, 33, 485, 1004, -100);

volatile int s13_out = 0;  // m1
volatile int s14_out = 0;  // m2

void setup() {
  
  Serial.begin(9600);
  Serial5.begin(115200);
 
  servo_13.attach(13);
  servo_14.attach(14);
  
  servo_13.writeMicroseconds( 1500 ); // m1
  servo_14.writeMicroseconds( 1500 ); // m2

  timedOut = true;

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

  delay(50);

  AS5043obj.begin();

  millisTock = millis();
  
}

void alive() {
  
    // show status
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    // record msg time
    milisLastMsg = millis();
    timedOut = false;
    timescont++;
} 

int serialstatus = 0;
uint16_t word1 = 0;
int debug = 0;

void commloop() {
  
  if (Serial.available()) {
   if (Serial.read() == 'r') {
    debug = 1;
   } 
   else {
    debug = 0; 
   }
  }
  
  // PROT FF AL HL HL CHKSUM = 10 bytes
  if (Serial5.available() >= 10){
    
    if (word1 != 0xFFFF) {
      while(Serial5.available()) {
        word1 = (word1 << 8) | Serial5.read();
        if (word1 == 0xFFFF) {
          if (Serial5.available() < 8) {
           delay(10);
          }
          break;
        }
      }
    }
    
    if (Serial5.available() < 8) {
      return; 
    }
    
    // read alive status
    uint16_t aldata = (Serial5.read() << 8) | Serial5.read();
    // read the servos data, in microseconds
    uint16_t s1data = (Serial5.read() << 8) | Serial5.read();
    uint16_t s2data = (Serial5.read() << 8) | Serial5.read();
    // verify the checksum
    uint16_t checksumH;
    uint16_t checksumL;
    //
    uint16_t localchecksumH = (((long)aldata + (long)s1data + (long)s2data) >> 8) & 255;
    uint16_t localchecksumL = (((long)aldata + (long)s1data + (long)s2data) >> 0) & 255;
    
    if (debug) {
     Serial.print(highByte(word1));
     Serial.print(" "); 
     Serial.print(lowByte(word1));
     Serial.print(" "); 
     Serial.print(highByte(aldata));
     Serial.print(" "); 
     Serial.print(lowByte(aldata));
     Serial.print(" "); 
     Serial.print(highByte(s1data));
     Serial.print(" "); 
     Serial.print(lowByte(s1data));
     Serial.print(" "); 
     Serial.print(highByte(s2data));
     Serial.print(" "); 
     Serial.print(lowByte(s2data));
     Serial.print(" "); 
     Serial.print(checksumH);
     Serial.print(" "); 
     Serial.print(checksumL);
     Serial.print(" local: "); 
     Serial.print(localchecksumH);
     Serial.print(" "); 
     Serial.print(localchecksumL);
     Serial.println(""); 
    }
    
    // TODO add more aldata rules
    if ((checksumH == localchecksumH) && (checksumL == localchecksumL)) {
      if (aldata == 1) {
        s13_out = s1data;
        s14_out = s2data;
        alive();
      }
    }
  }
}

void loop() {
  
  commloop();
  
  if (millisTock <= millis()) {
    
    millisTock += 50;
    
    if (millis() - milisLastMsg > 1500) {
      // shut down all
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
      s13_out = 0;  // m1
      s14_out = 0;  // m2
      timedOut = true;
      timescont = 0;
    }
    
    if (timedOut) {
      servo_13.writeMicroseconds( 1500 ); // m1
      servo_14.writeMicroseconds( 1500 ); // m2
    }
    else {
      servo_13.writeMicroseconds( s13_out / 20 + 1500); // m1 // 100 points would be a 1 percent, 5 of 500 us, min of 0.2 percent
      servo_14.writeMicroseconds( s14_out / 20 + 1500); // m2 // 100 points would be a 1 percent, 5 of 500 us, min of 0.2 percent
    }
  }
}
