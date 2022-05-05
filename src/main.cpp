#include <Arduino.h>
#include <TinyPICO.h>
#include <Tasker.h>
#include <ServoInput.h>
#include "crc8.h"
#include "torqeedo.h"

// Initialise the TinyPICO library
TinyPICO tp = TinyPICO();
// Initialize CRC8-Maxim calculation class
CRC8 crc8;
TorqeedoMotor leftMotor;

void ledOn(){
  tp.DotStar_SetPixelColor( 255, 128, 0 );
}
void ledOff(){
  tp.DotStar_Clear();
}

void setup() {
  ledOn();
  delay(2);
  ledOff();
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}