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

#define THROTLE_PIN 23
#define MIX_PIN 19



// Throttle
volatile unsigned long pulseInTimeBegin = micros();
volatile unsigned long pulseInTimeEnd = micros();
volatile bool newPulseDurationAvailable = false;
volatile int16_t throttleOrder = 0;


// Mix
volatile unsigned long mixInTimeBegin = micros();
volatile unsigned long mixInTimeEnd = micros();
volatile bool newMixDurationAvailable = false;
volatile int16_t mixOrder = 0;

void throttlePinInterrupt()
{
  if (digitalRead(THROTLE_PIN) == HIGH) {
    // start measuring
    pulseInTimeBegin = micros();
  }
  else {
    // stop measuring
    pulseInTimeEnd = micros();
    newPulseDurationAvailable = true;
  }
}

void mixPinInterrupt()
{
  if (digitalRead(MIX_PIN) == HIGH) {
    // start measuring
    mixInTimeBegin = micros();
  }
  else {
    // stop measuring
    mixInTimeEnd = micros();
    newMixDurationAvailable = true;
  }
}


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

  pinMode(THROTLE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(THROTLE_PIN),
                  throttlePinInterrupt,
                  CHANGE);

  pinMode(MIX_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MIX_PIN),
                  mixPinInterrupt,
                  CHANGE);


  ledOff();

  // Seting PWM PINS input
  // pinMode(TROTHLE_PIN, INPUT); 
  // pinMode(MIX_PIN, INPUT); 
  // delay(5000); // Wait 5 seconds and turn off motor
  // leftMotor.Off(); // Turn Off left motor
}

uint32_t startTime = millis();

void loop() {

  // delay(1);
  if (newPulseDurationAvailable) {
    newPulseDurationAvailable = false;
    unsigned long pulseDuration = pulseInTimeEnd - pulseInTimeBegin;
    throttleOrder = map(pulseDuration, 985, 2061, -1000, 1000);
    // Serial.print("pulseDuration : "); Serial.println(pulseDuration);
    // Serial.print("throttleOrder : "); Serial.println(throttleOrder);
  }

  if (newMixDurationAvailable) {
    newMixDurationAvailable = false;
    unsigned long mixDuration = mixInTimeEnd - mixInTimeBegin;
    mixOrder = map(mixDuration, 985, 2061, -1000, 1000);
    // Serial.print("mixDuration : "); Serial.println(mixDuration);
    // Serial.print("mixOrder : "); Serial.println(mixOrder);
  }

  leftMotor.loop(throttleOrder);
  rightMotor.loop(mixOrder);
  // put your main code here, to run repeatedly:
}