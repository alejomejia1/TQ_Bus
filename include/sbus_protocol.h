#ifndef SBUS_PROTOCOL_H
#define SBUS_PROTOCOL_H

#include <Arduino.h>
#include "HardwareSerial.h"

///////////////////////////////////////////SBUS DEFINITIONS//////////////////////////////////////////////////////////////
static const int8_t SBUS_BUF_LEN = 25;
/* SBUS message defs */
static constexpr int8_t NUM_SBUS_CH = 16;
static constexpr uint8_t SBUS_HEADER = 0x0F;
static constexpr uint8_t SBUS_FOOTER = 0x00;
//static constexpr uint8_t FOOTER2_ = 0x04;
static constexpr uint8_t CH17_MASK = 0x01;
static constexpr uint8_t CH18_MASK = 0x02;
static constexpr uint8_t LOST_FRAME_MASK = 0x04;
static constexpr uint8_t FAILSAFE_MASK = 0x08;

#define SBUS_TO_MOTORS_BOARD_PACKET_HEADER			0xAC    // communication packet header
#define SBUS_TO_MOTORS_BOARD_PACKET_FOOTER			0xAD    // communication packet footer
#define SBUS_TO_MOTORS_BOARD_PACKET_ESCAPE			0xAE    // escape character for handling occurrences of header, footer and this escape bytes in original message
#define SBUS_TO_MOTORS_BOARD_PACKET_ESCAPE_MASK	0x80    // byte after ESCAPE character should be XOR'd with this value
#define SBUS_TO_MOTORS_BOARD_FAILSAFE_MASK			0X08


class sbus_protocol
{
	public:
		sbus_protocol(HardwareSerial& ser);
		void begin(uint8_t rxPin, uint8_t txPin, bool INVERTED, uint32_t SBUSBAUD);
		void initVariables();
		void resetChannels();
		void msgReceived_slot();
		bool build_cmd_to_motors_board();
		bool pack_message_to_motors_board(unsigned char* msg_contents, int num_bytes);
		bool add_byte_to_message(unsigned char byte_to_add, unsigned char* msg_buff, int msg_buff_size, int16_t	&num_bytes);

		short ch[NUM_SBUS_CH];
		bool ch17;
		bool ch18;
		bool failsafe;
		bool lost_frame;
		int nrOfCompleteFrames;
		int nrOfVoidComms;

		int16_t send_buff_num_bytes;
		uint8_t send_buff[16];

	private:
		// int _ser;
		HardwareSerial* _ser;
		int _txPin;
		int _rxPin;
		uint32_t _sbusBaud = 100000;

		int rxState;
		unsigned char prevRxData;
		unsigned char buf[SBUS_BUF_LEN];


};
#endif // SBUS_PROTOCOL_H
