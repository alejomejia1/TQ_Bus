#ifndef TORQEEDO_MOTOR_CONTROL_H
#define TORQEEDO_MOTOR_CONTROL_H

#define INCLUDE_MOTORS_FUNCTIONS
#define INCLUDE_MOTOR0
#define INCLUDE_MOTOR1
#define INCLUDE_SBUS_RECEIVER
#define INCLUDE_PCF8574

#include <Arduino.h>
#include "HardwareSerial.h"
#include "crc8_table_maxim.h"

const int MAX_MOTORS=2;
const int MOTOR0=0;
const int MOTOR1=1;

const unsigned char INITIAL_REPETITIVE_FRAME_TO_TORQEEDO[]=	{0xac, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x8f, 0xad, 0xff};
const unsigned char ZERO_ORDER_FRAME_TO_TORQEEDO[]=					{0xac, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x81, 0xad, 0xff};

const unsigned char BUS_MASTER=0x00;

const int TORQEEDO_MESSAGE_LEN_MAX=35;
const unsigned char TORQEEDO_PACKET_HEADER=0xAC;
const unsigned char TORQEEDO_PACKET_FOOTER=0xAD;
const unsigned char TORQEEDO_PACKET_ESCAPE=0xAE;
const unsigned char TORQEEDO_PACKET_ESCAPE_MASK=0x80;

const int FULL_REVERSE=1000;
const int TWO_THIRD_REVERSE=666;
const int ONE_THIRD_REVERSE=333;
const int STOP_VALUE=0;
const int ONE_THIRD_FORWARD=333;
const int TWO_THIRD_FORWARD=666;
const int FULL_FORWARD=1000;

//MsgAddress
const unsigned char REMOTE_MESSAGE_ID=0x14;
const unsigned char LCD_MESSAGE_ID=0x20;

//DisplayMsgId
const unsigned char SYSTEM_STATE=0x41;
const unsigned char SYSTEM_SETUP=0x42;

const int NR_OF_REPETITIVE_INITIAL_ORDER_TO_MOTOR=10;
const int NR_OF_REPETITIVE_INITIAL_ZERO_ORDER_TO_MOTOR=20;

const int TIME_AFTER_MOTOR_ORDER_REQ_TO_SEND_ORDER_MS=1;

#define	SERIAL_PORT_BAUDRATE	19200


class torqeedo_motor_control
{

public:
	torqeedo_motor_control(){}
	void beginMotorSerial(uint8_t ser, uint8_t rxPin, uint8_t txPin, uint8_t onoff);
	void on();
	void off();
	void initVariables();
	void frameTransmission_slot(int motorId);
	bool build_motor_speed_cmd(int motorId);
	bool pack_message(unsigned char* msg_contents, int num_bytes);
	unsigned char crc8_maxim(unsigned char * mot_speed_cmd_buff, int mot_speed_cmd_buff_len);
	bool add_byte_to_message(unsigned char byte_to_add, unsigned char* msg_buff, int msg_buff_size, int	&num_bytes);
	short mapThrottleVar(unsigned short value);
	void msgReceived_slot(int connId);
	void parse_packet(int connId);

	int nrOfvoidComms;
	int motorDummyOrderCount;

	bool restartDelayTxTimer;
	bool txFlag;

	bool sBusCommsOk;
	short up_down_sbus_throttle_val;
	short left_right_sbus_throttle_val;
	short arm_button_val;

	typedef struct
	{
		uint16_t flags;									// flags, see above for individual bit definitions
		uint8_t master_state;           // deprecated
		uint8_t master_error_code;      // error code (0=no error)
		float motor_voltage;            // motor voltage in volts
		float motor_current;            // motor current in amps
		uint16_t motor_power;           // motor power in watts
		int16_t motor_rpm;              // motor speed in rpm
		uint8_t motor_pcb_temp;         // motor pcb temp in C
		uint8_t motor_stator_temp;      // motor stator temp in C
		uint8_t batt_charge_pct;        // battery state of charge (0 to 100%)
		float batt_voltage;             // battery voltage in volts
		float batt_current;             // battery current in amps
		uint16_t gps_speed;             // gps speed in knots * 100
		uint16_t range_miles;           // range in nautical miles * 10
		uint16_t range_minutes;         // range in minutes (at current speed and current draw)
		uint8_t temp_sw;                // master PCB temp in C (close to motor power switches)
		uint8_t temp_rp;                // master PCB temp in C (close to reverse voltage protection)
		uint32_t last_update_ms;        // system time that system state was last updated

	}display_system_state_;
	display_system_state_	display_system_state;

	typedef struct
	{
		uint8_t flags;              // 0 : battery config valid, all other bits unused
		uint8_t motor_type;         // motor type (0 or 3:Unknown, 1:Ultralight, 2:Cruise2, 4:Cruise4, 5:Travel503, 6:Travel1003, 7:Cruise10kW)
		uint16_t motor_sw_version;  // motor software version
		uint16_t batt_capacity;     // battery capacity in amp hours
		uint8_t batt_charge_pct;    // battery state of charge (0 to 100%)
		uint8_t batt_type;          // battery type (0:lead acid, 1:Lithium)
		uint16_t master_sw_version; // master software version
	}display_system_setup_;
	display_system_setup_	display_system_setup;

private:
	int _ser;
	int _txPin;
	int _rxPin;
	int _onOffPin;
	int _order;

	short _motor_speed_desired;
	short _motor_user_value;
	short _throttle_user_value;
	unsigned char _send_buff[TORQEEDO_MESSAGE_LEN_MAX];
	int _send_buff_num_bytes;

	unsigned char _rxFrame[256];
	int _rxFrameLen;
	bool _headerByteWasReceived;

	//int32_t _lastReqToMotorsMillis;

	
};
#endif // TORQEEDO_MOTOR_CONTROL_H
