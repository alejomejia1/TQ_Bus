#include <Arduino.h>
#include <TinyPICO.h>
#include <Tasker.h>
#include <ServoInput.h>
#include "crc8.h"
#include "torqeedo.h"


// // OTA Updater
// #include <WiFi.h>
// #include <WebServer.h>
// #include <ESP2SOTA.h>
// const char* ssid = "TIGO-5F09";
// const char* password = "4D9697503124";
// WebServer server(80);



// Initialise the TinyPICO library
TinyPICO tp = TinyPICO();
// Initialize CRC8-Maxim calculation class
CRC8 crc8;
// Initialize TorqeedoMotor class
TorqeedoMotor leftMotor;
TorqeedoMotor rightMotor;

#define THROTLE_PIN 23
#define MIX_PIN 19
#define ARM_PIN 18



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

// Arm
volatile unsigned long armInTimeBegin = micros();
volatile unsigned long armInTimeEnd = micros();
volatile bool newArmDurationAvailable = false;
volatile int16_t armOrder = 0;

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

void armPinInterrupt()
{
  if (digitalRead(ARM_PIN) == HIGH) {
    // start measuring
    armInTimeBegin = micros();
  }
  else {
    // stop measuring
    armInTimeEnd = micros();
    newArmDurationAvailable = true;
  }
}


void ledOn(){
  tp.DotStar_SetPixelColor( 255, 128, 0 );
}
void ledOff(){
  tp.DotStar_Clear();
}
void ledColor(int8_t r, int8_t g, int8_t b, int8_t i){
  tp.DotStar_SetPixelColor(r, g, b);
}



uint32_t startup_timer = millis();

void setup() {

  delay(500);

  Serial.begin(115200);

  Serial.println("Turning on motor");
  
  int16_t startTimePulse = millis();

  ledOn(); 

  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //     delay(500);
  //     Serial.print(".");
  // }
  // Serial.println("");
  // Serial.println("WiFi connected");
  // Serial.println("IP address: ");
  // Serial.println(WiFi.localIP());

  // /* SETUP YOR WEB OWN ENTRY POINTS */
  // server.on("/myurl", HTTP_GET, []() {
  //   server.sendHeader("Connection", "close");
  //   server.send(200, "text/plain", "Hello there!");
  // });

  // /* INITIALIZE ESP2SOTA LIBRARY */
  // ESP2SOTA.begin(&server);
  // server.begin();


  leftMotor.begin(1, 26, 25, 14, 32); // Serial 1, Tx 26, Rx 25, Rts 14, OnOff 32)
  rightMotor.begin(2, 15, 27, 4, 33); // Serial 2, Tx 15, Rx 27, Rts  4, OnOff 33)

  leftMotor.On();   // Turn On left motor
  leftMotor.send_tiller_init(); 

  rightMotor.On();  // Turn On right motor
  rightMotor.send_tiller_init(); 


  // Enable interrupts for control inputs

  // Throtle power order
  pinMode(THROTLE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(THROTLE_PIN),
                  throttlePinInterrupt,
                  CHANGE);

  // Mix for heading control
  pinMode(MIX_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MIX_PIN),
                  mixPinInterrupt,
                  CHANGE);
  

  // Arm / Disarm motor
  pinMode(ARM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ARM_PIN),
                  armPinInterrupt,
                  CHANGE);
  
}

uint32_t startTime = millis();


uint8_t received_buff[10];

// process a single byte received on serial port (In order to receive orders by serial protocol)
// return true if a complete message has been received (the message will be held in _received_buff)
// HEADING_CHAR $
// MOTOR_DESTINATION 1: LEFT, 2: RIGHT, 3: BOTH
// ORDER: +1000 :: -1000 (MUST INCLUDE NEGATIVE/POSITIVE SIGN)
// TERMINATION_CHAR ;

bool parse_cmd_byte(uint8_t b)
{
    
    bool complete_msg_received = false;
    uint8_t parse_state = 0;
    uint8_t parse_error_count = 0;
    uint8_t received_buff_len = 0;

    switch (parse_state) {
    case 0: // wait for init
        if (b == '$') {
            parse_state = 1;
        }
        received_buff_len = 0;
        break;
    case 1: // waiting for footer
        if (b == ';') {
            parse_state = 0;

            // check message length
            if (received_buff_len == 0) {
                parse_error_count++;
                break;
            }

            complete_msg_received = true;
            // Serial.println(_received_buff_len);

        } else {
            // add to buffer
            received_buff[received_buff_len] = b;
            received_buff_len++;
            if (received_buff_len > 10) {
                // message too long
                parse_state = 0;
                parse_error_count++;
            }
        }
        break;
    }

    return complete_msg_received;
}




void loop() {

  if (millis() - startup_timer > 5000) {
    ledColor(0,255,0,1);
  }
  // delay(1);
  if (newPulseDurationAvailable) {
    newPulseDurationAvailable = false;
    unsigned long pulseDuration = pulseInTimeEnd - pulseInTimeBegin;
    throttleOrder = map(pulseDuration, 985, 2061, -1000, 1000);
    if (throttleOrder > -60 && throttleOrder < 60) {
      throttleOrder = 0;
    }
    // Serial.print("pulseDuration : "); Serial.println(pulseDuration);
    // Serial.print("throttleOrder : "); Serial.println(throttleOrder);
  }

  if (newMixDurationAvailable) {
    newMixDurationAvailable = false;
    unsigned long mixDuration = mixInTimeEnd - mixInTimeBegin;
    mixOrder = map(mixDuration, 985, 2061, -1000, 1000);
    if (mixOrder > -70 && mixOrder < 70) {
      mixOrder = 0;
    }
    // Serial.print("mixDuration : "); Serial.println(mixDuration);
    // Serial.print("mixOrder : "); Serial.println(mixOrder);
  }


  if (newArmDurationAvailable) {
    newArmDurationAvailable = false;
    unsigned long armDuration = armInTimeEnd - armInTimeBegin;
    armOrder = map(armDuration, 985, 2061, -1000, 1000);
    Serial.print("armDuration : "); Serial.println(armDuration);
    Serial.print("armOrder : "); Serial.println(armOrder);
  }

  uint32_t nbytes = Serial.available();
  while (nbytes-- > 0) {
    int16_t b = Serial.read(); 
    if (b >= 0 ) {
      if (parse_cmd_byte((uint8_t)b)) {
          // complete message received, parse it!
          Serial.println("Comando recibido");
      }
    }
  }



  leftMotor.loop(throttleOrder + mixOrder);
  rightMotor.loop(throttleOrder -  mixOrder);
  // put your main code here, to run repeatedly:

  // /* HANDLE UPDATE REQUESTS */
  // server.handleClient();

}