#include <Adafruit_NeoPixel.h>
#include <avr/power.h>

#define PIN            6
#define NUMPIXELS      16
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DESPIERTO      220
#define TRISTE         400
#define COLORFELIZ     214,252,23
#define COLORDESPIERTO 87,209,227
#define COLORTRISTE    255,64,0

void setup() {
  pixels.begin();
  Serial.begin(57600);
}

float filtrado;

void loop() {
  float val  = analogRead(A0);
  filtrado = val * 0.01 + filtrado * 0.99;
  
  if (filtrado < DESPIERTO)
    feliz();
  if (filtrado > DESPIERTO && filtrado < TRISTE)
    despierto();
  if (filtrado > TRISTE)
    triste();
    
  delay(10);
  
  Serial.println(filtrado);
}

void feliz(){
 for(int i=0;i<16;i++){
    if (i > 4 && i < 12)
      pixels.setPixelColor(i, pixels.Color(0, 0,0)); 
    else
      pixels.setPixelColor(i, pixels.Color(COLORFELIZ)); 
  }
  pixels.show(); // This sends the updated pixel color to the hardware. 
}

void despierto() {
  for(int i=0;i<16;i++){
    pixels.setPixelColor(i, pixels.Color(COLORDESPIERTO)); 
  }
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void triste() {
  for(int i=0;i<16;i++){
    if (i > 4 && i < 12)
      pixels.setPixelColor(i, pixels.Color(COLORTRISTE));
    else
      pixels.setPixelColor(i, pixels.Color(0, 0,0));
  }
  pixels.show(); // This sends the updated pixel color to the hardware.
}
