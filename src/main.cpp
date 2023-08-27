#include <Arduino.h>
// #include <TinyPICO.h>
#include <Tasker.h>
#include <ServoInput.h>
#include "crc8.h"
#include "torqeedo.h"


#include <HardwareBLESerial.h>

HardwareBLESerial &bleSerial = HardwareBLESerial::getInstance();
// // OTA Updater
// #include <WiFi.h>
// #include <WebServer.h>
// #include <ESP2SOTA.h>
// const char* ssid = "TIGO-5F09";
// const char* password = "4D9697503124";
// WebServer server(80);



// Initialise the TinyPICO library
// TinyPICO tp = TinyPICO();
// Initialize CRC8-Maxim calculation class
CRC8 crc8;
// Initialize TorqeedoMotor class
TorqeedoMotor motor;

#define THROTLE_PIN 10
#define BACKLIGHT 38
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
int8_t ledState = LOW;

// Variables to store serial input
char receivedChar;
bool newData = false;

// Variables to store parsed message
char direction;
int speed;



void ledOn(){
  // tp.DotStar_SetPixelColor( 255, 128, 0 );
  digitalWrite(BACKLIGHT, HIGH);
}
void ledOff(){
  // tp.DotStar_Clear();
  digitalWrite(BACKLIGHT, LOW);
}
void ledColor(int8_t r, int8_t g, int8_t b, int8_t i){
  // tp.DotStar_SetPixelColor(r, g, b);
}



uint32_t startup_timer = millis();

void setup() {

  delay(1000);

  Serial.begin(115200);


  Serial.println("Turning on motor");



  int16_t startTimePulse = millis();

  pinMode(BACKLIGHT, OUTPUT);

  ledOn();

  if (!bleSerial.beginAndSetupBLE("TQ BLE")) {
    while (true) {
      Serial.println("failed to initialize HardwareBLESerial!");
      delay(1000);
    }
  }

  Serial.println("HardwareBLESerial central device connected!");

  motor.begin(1, 1, 2, 3); // Serial 1, Tx 26, Rx 25, OnOff 32)
  // rightMotor.begin(2, 15, 27, 4, 33); // Serial 2, Tx 15, Rx 27, OnOff 33)

  motor.On();   // Turn On left motor
  motor.send_tiller_init();

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
            Serial.println("Data received");

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

void parseSerialMessage()
{
    static String message;
    while (Serial.available()) {
        char ch = Serial.read();

        if (ch == '$') {
          Serial.println(); // Start of message
          message = "";
        }
        else if (ch == ';') {   // End of message

          if (message.length() >= 5) {
            Serial.println(message);
            receivedChar = message.charAt(0);      // Direction (F or R)
            direction = receivedChar;
            speed = message.substring(1, 5).toInt();  // Speed (4 digits)
            newData = true;
          }
          message = "";
        }
        else {
          message += ch;
        }
    }
}

void parseSerialBLEMessage()
{
    static String message;
    while (bleSerial.available()) {
        char ch = bleSerial.read();

        if (ch == '$') {
          message = "";
        }
        else if (ch == ';') {   // End of message

          if (message.length() >= 5) {
            receivedChar = message.charAt(0);      // Direction (F or R)
            direction = receivedChar;
            speed = message.substring(1, 5).toInt();  // Speed (4 digits)
            newData = true;
          }
          message = "";
        }
        else {
          message += ch;
        }
    }
}

void processSerialMessage()
{
  // Perform actions based on the received message
  if (direction == 'F') {
    // Move forward with the specified speed
    // Your code here...
    throttleOrder = speed;
  } else if (direction == 'R') {
    // Move reverse with the specified speed
    // Your code here...
    throttleOrder = -speed;
  } else if (direction == 'S') {
    // Move reverse with the specified speed
    // Your code here...
    throttleOrder = 0;
  }

  // Display the received message
  Serial.print("Received Message: ");
  Serial.print(direction);
  Serial.print(",");
  Serial.println(speed);
}


void loop() {

  if (millis() - startup_timer > 1000) {
    // toggle state of LED
    ledState = !ledState;
    digitalWrite(BACKLIGHT, ledState);
    // Serial.print("ledState : "); Serial.println(ledState);
    startup_timer = millis();
    // ledColor(0,255,0,1);

  }

  // this must be called regularly to perform BLE updates
  bleSerial.poll();

  parseSerialMessage();
  parseSerialBLEMessage();

  if (newData) {
    // Process the received message
    processSerialMessage();

    // Reset the newData flag
    newData = false;
  }




  motor.loop(throttleOrder);
  // rightMotor.loop(throttleOrder -  mixOrder);
  // put your main code here, to run repeatedly:

  // /* HANDLE UPDATE REQUESTS */
  // server.handleClient();

}