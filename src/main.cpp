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
TorqeedoMotor rightMotor;

// Steering Setup
const int SteeringSignalPin = 18;  // MUST be interrupt-capable!
const int SteeringPulseMin = 1000;  // microseconds (us)
const int SteeringPulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" example

// ServoInputPin<SteeringSignalPin> steering(SteeringPulseMin, SteeringPulseMax);

// Throttle Setup
const int ThrottleSignalPin = 23;  // MUST be interrupt-capable!
const int ThrottlePulseMin = 1000;  // microseconds (us)
const int ThrottlePulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" example

// ServoInputPin<ThrottleSignalPin> throttle(ThrottlePulseMin, ThrottlePulseMax);


void ledOn(){
  tp.DotStar_SetPixelColor( 255, 128, 0 );
}
void ledOff(){
  tp.DotStar_Clear();
}

void setup() {

  delay(500);

  Serial.begin(115200);

  Serial.println("Turning on motor");
  

  int16_t startTimePulse = millis();

  ledOn(); 

  leftMotor.begin(1, 26, 25, 14, 32); // Serial 1, Tx 26, Rx 25, Rts 14, OnOff 32)
  rightMotor.begin(2, 15, 27, 4, 33); // Serial 2, Tx 15, Rx 27, Rts  4, OnOff 33)

  leftMotor.On();   // Turn On left motor
  rightMotor.On();  // Turn On right motor

  ledOff();

  pinMode(23, INPUT); 
  // delay(5000); // Wait 5 seconds and turn off motor
  // leftMotor.Off(); // Turn Off left motor
}

uint32_t startTime = millis();

void loop() {
  
  int16_t throttleOrder;
  // int16_t channel;
  // channel = pulseIn(23,HIGH);
  // int16_t throttleOrder = map(channel, 998, 2000, -1000, 1000);
  // Serial.print("Pulse:"); Serial.println(throttleOrder);

  // if (throttleOrder > -50 && throttleOrder < 50) {
  //   throttleOrder = 0;
  // }

  // if ((millis() - startTime < 5000) ) {
  //   throttleOrder = 400;
  // } 
  // if ((millis() - startTime > 5000) && (millis() - startTime < 6000) ) {
  //   throttleOrder = 0; 
  // }
  // if ((millis() - startTime > 6000) && (millis() - startTime < 11000) ) {
  //   throttleOrder = -500;
  // }
  // if ((millis() - startTime > 11000) && (millis() - startTime < 12000) ) {
  //   throttleOrder = 0; 
  // } 
  // if ((millis() - startTime > 12000) ) {
  //   startTime = millis();
  // } 

  // delay(1);

  leftMotor.loop();
  rightMotor.loop();
  // put your main code here, to run repeatedly:
}