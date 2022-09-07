#include <Arduino.h>
// #include <TinyPICO.h>
// #include <Tasker.h>
// #include <ServoInput.h>
// #include <EasyNeoPixels.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>               // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h"        // legacy: #include "SSD1306.h"
#include "sbus_protocol.h"

#define LED_PIN    2
// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 14
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);


SSD1306Wire display(0x3c, SDA, SCL); 


sbus_protocol sbus(Serial);


#include "torqeedo_motor_control.h"

#include "sbus_from_master_control_board_externals.h"

bool printMessage=false;

// Initialize the TinyPICO library
// TinyPICO tp = TinyPICO();

void ledOn(uint8_t idx, uint8_t R, uint8_t G ,uint8_t B, uint8_t W)
{
	strip.setBrightness(W);
	strip.setPixelColor(idx, strip.Color(R, G, B));
	strip.show(); // Initialize all pixels to 'off'
}



void ledOff(uint8_t idx)
{
	strip.setPixelColor(idx, strip.Color(0, 0, 0));
	strip.show(); // Initialize all pixels to 'off'
}

torqeedo_motor_control motor[MAX_MOTORS];

bool sendMessage=false;

hw_timer_t *delayTxTimer0;
hw_timer_t *delayTxTimer1;
hw_timer_t *mainTimer;

long timeLastLeft = 0;
long timeLastRight = 0;

//===========================================================delayTxTimer======================================================================
#ifdef INCLUDE_MOTORS_FUNCTIONS
void IRAM_ATTR frameTransmission_slot_mot0() 
{
	timerStop(delayTxTimer0);
	motor[0].txFlag=true;
}

void IRAM_ATTR frameTransmission_slot_mot1() 
{
	timerStop(delayTxTimer1);
	motor[1].txFlag=true;
}

void setupTxTimer0() 
{
	delayTxTimer0 = timerBegin(0, 80, true);
	timerAttachInterrupt(delayTxTimer0, &frameTransmission_slot_mot0, true);
	timerAlarmWrite(delayTxTimer0, (TIME_AFTER_MOTOR_ORDER_REQ_TO_SEND_ORDER_MS * 1000), true);	//1 ms 
	timerAlarmEnable(delayTxTimer0);
	timerStop(delayTxTimer0);

	motor[0].restartDelayTxTimer=false;
}

void setupTxTimer1() 
{
	//delayTxTimer1=NULL;
	delayTxTimer1 = timerBegin(1, 80, true);
	timerAttachInterrupt(delayTxTimer1, &frameTransmission_slot_mot1, true);
	timerAlarmWrite(delayTxTimer1, (TIME_AFTER_MOTOR_ORDER_REQ_TO_SEND_ORDER_MS * 1000), true);	//1 ms 
	timerAlarmEnable(delayTxTimer1);
	timerStop(delayTxTimer1);

	motor[1].restartDelayTxTimer=false;
}
#endif

//=========================================================mainTimer=============================================================================
int32_t ledTimeout=0;
bool ledState=false;

void IRAM_ATTR mainTimerTimeout_slot()	//each 100ms
{
	sendMessage=true;
	
	#ifdef INCLUDE_MOTORS_FUNCTIONS
	for (int i=0; i<MAX_MOTORS; i++)
	{
		if (motor[i].nrOfvoidComms<9)
			motor[i].nrOfvoidComms++;
		else if (motor[i].nrOfvoidComms==9)
		{
			motor[i].motorDummyOrderCount=0;
			motor[i].nrOfvoidComms=10;
		}
	}
	#endif

	if (nrOfVoidSbusComms<9)
	{
		#ifdef INCLUDE_MOTORS_FUNCTIONS
		motor[0].sBusCommsOk=true;
		motor[0].up_down_sbus_throttle_val=sBusCh[0];
		motor[0].left_right_sbus_throttle_val=sBusCh[1];
		motor[0].arm_button_val=sBusCh[2];
		
		motor[1].sBusCommsOk=true;
		motor[1].up_down_sbus_throttle_val=sBusCh[0];
		motor[1].left_right_sbus_throttle_val=sBusCh[1];
		motor[1].arm_button_val=sBusCh[2];
		#endif
		nrOfVoidSbusComms++;
	}
	else if (nrOfVoidSbusComms==9)
	{
		Serial.println("sbus protocol comms lost");
		sBusCh[0]=1072;
		sBusCh[1]=1072;
		sBusCh[2]=1072;
		sBusFailsafe=false;
		nrOfVoidSbusComms=10;
	}
	else
	{
		#ifdef INCLUDE_MOTORS_FUNCTIONS
		motor[0].sBusCommsOk=false;
		motor[1].sBusCommsOk=false;
		#endif
	}		

	if (ledTimeout<4)
		ledTimeout++;
	else
	{
		ledTimeout=0;
		if (ledState==true)
		{
			ledState=false;
			// ledOff(0);	
		}
		else
		{
			ledState=true;
			// ledOn(0,0,0,60);
		}
	}
}

void setupMainTimer() 
{
	mainTimer = timerBegin(3, 80, true);
	timerAttachInterrupt(mainTimer, &mainTimerTimeout_slot, true);
	timerAlarmWrite(mainTimer, 100000, true);	//200 ms, continue counting 
	timerAlarmEnable(mainTimer);
}

void setup() 
{
	// setupEasyNeoPixels(2, 6);

	strip.begin();
	strip.setBrightness(50);
	
	// Yellow leds, Initializing
	ledOn(0,160,160,0,50);
	ledOn(1,160,160,0,50);

	// External leds (TODO:       Define number and function)
	ledOn(2,160,0,0,50);
	ledOn(3,0,160,0,50);
	ledOn(4,0,0,160,50);
	ledOn(5,160,0,0,50);
	ledOn(6,0,160,0,50);
	ledOn(7,0,0,160,50);
	ledOn(8,160,160,0,50);
	ledOn(9,0,160,160,50);
	ledOn(10,160,0,160,50);
	ledOn(11,160,160,160,50);
	ledOn(12,0,160,0,50);
	ledOn(13,0,0,160,50);

	// Initialising the UI will init the display too.
	display.init();

	display.flipScreenVertically();
	display.setFont(ArialMT_Plain_24);
	display.drawString(5, 12, "Travel 1103");
	display.setFont(ArialMT_Plain_16);
	display.drawString(15, 40, "Motor Control");
	display.display();

	Serial.begin(57600, SERIAL_8N1, 21, 23);	//rx 21, tx 23
	Serial.println("Torqeedo \n");
	
	// ledOn(0,60,60,0); 
	
	#ifdef INCLUDE_MOTORS_FUNCTIONS
		#ifdef INCLUDE_MOTOR0
		//esta forma es usando el integrado RS485 a TTL con autogestión de RTS
		motor[0].beginMotorSerial(1, 26, 25, 32); 	// Serial 1, Rx 26, Tx 25, OnOff 32	
		motor[0].initVariables();
		#endif

		#ifdef INCLUDE_MOTOR1
		//esta forma es usando el integrado RS485 a TTL con autogestión de RTS
		motor[1].beginMotorSerial(2, 15, 27, 33); // Serial 2, Rx 15, Tx 27, OnOff 33
		motor[1].initVariables();
		#endif

		#ifdef INCLUDE_MOTOR0
		motor[0].on();	// Turn on motor
		#endif

		#ifdef INCLUDE_MOTOR1
		motor[1].on();	// Turn on motor
		#endif
	#endif

	#ifdef INCLUDE_SBUS_RECEIVER
		sbus.begin(23, 19, true , 100000);	// Serial 0, Rx 23, Tx 19, inverted for sbus
		sbus.initVariables();
		sbus.resetChannels();
	#endif
	
	delay(1000);
	ledOff(0);
	ledOff(1);
	
	#ifdef INCLUDE_MOTORS_FUNCTIONS
	#ifdef INCLUDE_MOTOR0
	setupTxTimer0();
	#endif
	#ifdef INCLUDE_MOTOR1
	setupTxTimer1();
	#endif
	#endif

	setupMainTimer();
}



void loop() 
{
	// clear the display
 	// 
	// timeLastLeft = millis();
	// timeLastRight = millis();

	static int count=0;
	if (sendMessage)
	{
		/*
		Serial.print("Sbus chs=");
		for (int i=0; i<3; i++)
		{
			Serial.print(int (sBusCh[i]));
			Serial.print(" ");
		}
		Serial.print("FS=");
		Serial.print(sBusFailsafe);
		Serial.println("");
		*/
		
		//conform motors info transmission
		unsigned char motorsInfoBuff[256];
		int motorsInfoLen=0;
		float tempFloat;
		short tempShort;
		
		for (int i=0; i<2; i++)
		{
			if (motor[i].nrOfvoidComms==10)
			{
				for (int j=0; j<9; j++)
					motorsInfoBuff[motorsInfoLen++]=0xff;
			}
			else
			{
				tempFloat=motor[i].display_system_state.motor_voltage;
				tempFloat*=100;
				tempShort=tempFloat;
				motorsInfoBuff[motorsInfoLen++]=highByte(tempShort);
				motorsInfoBuff[motorsInfoLen++]=lowByte(tempShort);
				
				tempFloat=motor[i].display_system_state.motor_current;
				tempFloat*=10;
				tempShort=tempFloat;
				motorsInfoBuff[motorsInfoLen++]=highByte(tempShort);
				motorsInfoBuff[motorsInfoLen++]=lowByte(tempShort);

				tempShort=motor[i].display_system_state.motor_rpm;
				motorsInfoBuff[motorsInfoLen++]=highByte(tempShort);
				motorsInfoBuff[motorsInfoLen++]=lowByte(tempShort);

				motorsInfoBuff[motorsInfoLen++]=motor[i].display_system_state.batt_charge_pct;

				tempShort=motor[i].display_system_state.gps_speed;
				motorsInfoBuff[motorsInfoLen++]=highByte(tempShort);
				motorsInfoBuff[motorsInfoLen++]=lowByte(tempShort);
			}
		}
		
		unsigned char realBuff[256];
		int realBuffLen=0;
		unsigned char motorsInfoByte;
		
		realBuff[realBuffLen++]=0xAC;	//include header
		for (int i=0; i<motorsInfoLen; i++)	//include each byte of motors info
		{
			motorsInfoByte = motorsInfoBuff[i];
			if (motorsInfoByte==0xAC || motorsInfoByte==0xAD || motorsInfoByte==0xAE)
			{
				realBuff[realBuffLen++] = 0xAE;	//escape
				realBuff[realBuffLen++] = motorsInfoByte ^ 0x80;	//escape mask
			}
			else
				realBuff[realBuffLen++] = motorsInfoByte;
		}
		realBuff[realBuffLen++]=0xAD;	//footer

		for (int i=0; i<realBuffLen; i++)
		{
			Serial.printf("%c", realBuff[i]);
		}
		sendMessage=false;
	}

#ifdef INCLUDE_MOTORS_FUNCTIONS
	for (int i=0; i<2; i++)
	{
		if (motor[i].restartDelayTxTimer==true)
		{
			if (i==0)
				timerStart(delayTxTimer0);
			else
				timerStart(delayTxTimer1);
			motor[i].restartDelayTxTimer=false;
		}
		else if (motor[i].txFlag==true)
		{
			motor[i].frameTransmission_slot(i);
		}
	}
	
	int bytesToRead;
	
#ifdef INCLUDE_MOTOR0
	bytesToRead=Serial1.available();
	if (bytesToRead)
	{
		motor[0].msgReceived_slot(0);
		ledOn(1,0,60,0,50);
		timeLastLeft = millis();
	} 
	if (millis()-timeLastLeft > 1000) {
		ledOn(1,60,0,0,50);
	}
#endif
#ifdef INCLUDE_MOTOR1
	bytesToRead=Serial2.available();
	if (bytesToRead)
	{
		motor[1].msgReceived_slot(1);
		ledOn(0,0,60,0,50);
		timeLastRight = millis();
	}
	if (millis()-timeLastRight > 1000) {
		ledOn(0,60,0,0,50);
	}
	#endif
#endif

	//SBUS INFO FROM MASTER CONTROL BOARD
	if (Serial.available())
		sbus.msgReceived_slot();
	// bytesToRead=Serial.available();
	// if (bytesToRead)
	// {
	// 	decode_sbus_from_master_info();
	// }
}

