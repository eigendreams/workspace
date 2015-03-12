#include "Servo.h"
#include "AS5043.h"
#include "Encoder.h"

unsigned long milisLastMsg = 0;
unsigned long millisTock = 0;
bool          timedOut = false;

Servo servo_13;
Servo servo_14;

volatile int s13_out = 0;  // m1
volatile int s14_out = 0;  // m2

void setup() {
  
  Serial.begin(115200);
  Serial5.begin(115200);
 
  servo_13.attach(13);  // m1
  servo_14.attach(14);  // m2
  
  servo_13.writeMicroseconds( 1500 ); // m1
  servo_14.writeMicroseconds( 1500 ); // m2

  timedOut = true;

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  
  millisTock = millis();
}

uint16_t start = 0;
int      debug = 0;
int      byte1 = 0;
int      byte2 = 0;

void commloop() {
  
  if (Serial.available()) {
   if (Serial.read() == '1') {
    debug = 1;
   } 
   else {
    debug = 0; 
   }
  }
  
  // PROT FF AL HL HL CHKSUM = 10 bytes, 80 bits -> 694.4 us per string
  // assuming 115200 bauds, faster may risk noise errors and such
  if (Serial5.available() >= 10){
    
    // 0x7530 = 30000
    // 30000 es imposible como checksum
    // los valores de los servos DEBEN estar entre -10000 y 10000
    // Las siguientes lineas permiten consumir el buffer hasta hallar
    // el punto de inicio correcto sin desperdiciar parte de la trama
    // it's preaty neat, in my opinion
    byte1 = Serial5.read();
    start = (start << 8) | byte1;
    if (start != 0x7530) {
      while(Serial5.available()) {
        byte1 = Serial5.read();
        start = (start << 8) | byte1;
        if (start == 0x7530) {
          if (Serial5.available() < 8) {
           delay(10); // wait for the next data string
          }
          break;
        }
      }
    }
    
    if (Serial5.available() < 8) {
      return; // maybe the communication broke? the program stopped? the system crashed?
    }
    
    // read alive status
    byte1 = Serial5.read();
    byte2 = Serial5.read();
    uint16_t aldata = (byte1 << 8) | byte2;
    // read the servos data, in microseconds, SIGNED
    byte1 = Serial5.read();
    byte2 = Serial5.read();
    int16_t s1data = (byte1 << 8) | byte2;
    // read the servos data, in microseconds, SIGNED
    byte1 = Serial5.read();
    byte2 = Serial5.read();
    int16_t s2data = (byte1 << 8) | byte2;
    // read the checksum
    byte1 = Serial5.read();
    byte2 = Serial5.read();
    uint16_t checksum = (byte1 << 8) | byte2;

    // calculate the checksum
    uint16_t localchecksum = ((long)aldata + (long)s1data + (long)s2data);
    
    if (debug == 1) {
      // Consumes up to 536 bits or 4.652 ms at 115200 bauds
     Serial.print(" id "); 
     Serial.print(start);
     Serial.print(" al "); 
     Serial.print(aldata);
     Serial.print(" s1 "); 
     Serial.print(s1data);
     Serial.print(" s2 "); 
     Serial.print(s2data);
     Serial.print(" ch "); 
     Serial.print(checksum);;
     Serial.print(" || sh "); 
     Serial.print(localchecksum);
     Serial.print(" bf "); 
     Serial.print(Serial5.available());
     Serial.println(""); 
    }
    
    // TODO add more aldata rules
    if (checksum == localchecksum) {
      if (aldata == 1) {
        s13_out = s1data;
        s14_out = s2data;
        // show status in LEDs
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
        // record msg time
        milisLastMsg = millis();
        timedOut = false;
      }
    }
  }
}

void loop() {
  
  commloop();
  
  if (millisTock <= millis()) {

    millisTock += 40;
    
    if ((millis() - milisLastMsg) > 1000) {
      // shut down everything
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
      s13_out = 0;  // m1
      s14_out = 0;  // m2
      timedOut = true;
    }
    
    if (timedOut) {
      servo_13.writeMicroseconds( 1500 ); // m1
      servo_14.writeMicroseconds( 1500 ); // m2
    }
    else {
      servo_13.writeMicroseconds( s13_out / 20 + 1500 ); // m1 // 100 points would be a 1 percent, 5 of 500 us, min of 0.2 percent
      servo_14.writeMicroseconds( s14_out / 20 + 1500 ); // m2 // 100 points would be a 1 percent, 5 of 500 us, min of 0.2 percent
    }
  }
}
