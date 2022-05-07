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

// Steering Setup
const int SteeringSignalPin = 19;  // MUST be interrupt-capable!
const int SteeringPulseMin = 1000;  // microseconds (us)
const int SteeringPulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<SteeringSignalPin> steering(SteeringPulseMin, SteeringPulseMax);

// Throttle Setup
const int ThrottleSignalPin = 23;  // MUST be interrupt-capable!
const int ThrottlePulseMin = 1000;  // microseconds (us)
const int ThrottlePulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<ThrottleSignalPin> throttle(ThrottlePulseMin, ThrottlePulseMax);


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
  int16_t throttleOrder = throttle.map(-1000, 1000);  // remap to a percentage both forward and reverse
  // Serial.print("Throttle: ");Serial.println(throttleOrder);
  if (throttleOrder > -100 && throttleOrder < 100) {
    throttleOrder = 0;
  }
  if ((millis() - startTime < 20000) ) {
    
    leftMotor.loop(throttleOrder);
  } else {
    leftMotor.Off();
  }
  // put your main code here, to run repeatedly:
}