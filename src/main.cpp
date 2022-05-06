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
// Initialize TorqeedoMotor class
TorqeedoMotor leftMotor;

void ledOn(){
  tp.DotStar_SetPixelColor( 255, 128, 0 );
}
void ledOff(){
  tp.DotStar_Clear();
}

void setup() {

  delay(2000);

  Serial.begin(115200);

  Serial.println("Turning on motor");
  

  ledOn(); 

  leftMotor.begin(1, 26, 25, 14, 32); // Serial Number, 
  leftMotor.On();   // Turn On left motor

  ledOff();

  // delay(5000); // Wait 5 seconds and turn off motor
  // leftMotor.Off(); // Turn Off left motor
}

uint32_t startTime = millis();

void loop() {
  

  if ((millis() - startTime < 10000) ) {
    leftMotor.loop();
  } else {
    leftMotor.Off();
  }
  // put your main code here, to run repeatedly:
}