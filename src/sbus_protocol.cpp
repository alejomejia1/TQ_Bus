#include "sbus_protocol.h"
/* SBUS object, input the serial bus */
sbus_protocol::sbus_protocol(HardwareSerial& ser) {
	_ser = &ser;
}

/* starts the serial communication */
void sbus_protocol::begin(uint8_t rxPin, uint8_t txPin, bool INVERTED, uint32_t SBUSBAUD)	
{
	_txPin = txPin;
	_rxPin = rxPin;
	_sbusBaud = SBUSBAUD;
	_ser->begin(_sbusBaud, SERIAL_8E2, rxPin, txPin, INVERTED);
}

/* set Variables initial state */
void sbus_protocol::initVariables()
{
	rxState=0;
	prevRxData=0xff;
	nrOfVoidComms=5;
	int send_buff_num_bytes=0;
}


void sbus_protocol::resetChannels()
{
	for (int i=0; i<NUM_SBUS_CH; i++)
	{
		ch[i]=1072;	//sbus limits are [172:1972]. Mid value=1072
	}
	nrOfCompleteFrames=0;
}

void sbus_protocol::msgReceived_slot()
{
	int bytesToRead;
	unsigned char rxData;
	bytesToRead=Serial.available();
	if (bytesToRead)
	{
		for (int i=0; i<bytesToRead; i++)
		{
			rxData=Serial.read();
			if (prevRxData==SBUS_FOOTER && rxData==SBUS_HEADER)
			{
				//Serial.println("HEADER DETECTED");
				rxState=0;
			}
			if (rxState < SBUS_BUF_LEN-1)
			{
				buf[rxState++]=rxData;
			}
			else
			{
				if (rxData==SBUS_FOOTER)
				{
					//Serial.print("SBUS PROT COMPLIED nr ");
					//Serial.print(nrOfCompleteFrames++);
					ch[0]  = static_cast<int16_t>(	(buf[1]       | buf[2]  << 8) & 0x07FF);
					ch[1]  = static_cast<int16_t>(	(buf[2]  >> 3 | buf[3]  << 5) & 0x07FF);
					ch[2]  = static_cast<int16_t>(	(buf[3]  >> 6 | buf[4]  << 2  | buf[5] << 10) & 0x07FF);
					ch[3]  = static_cast<int16_t>(	(buf[5]  >> 1 | buf[6]  << 7) & 0x07FF);
					ch[4]  = static_cast<int16_t>(	(buf[6]  >> 4 | buf[7]  << 4) & 0x07FF);
					ch[5]  = static_cast<int16_t>(	(buf[7]  >> 7 | buf[8]  << 1  | buf[9] << 9) & 0x07FF);
					ch[6]  = static_cast<int16_t>(	(buf[9]  >> 2 | buf[10] << 6) & 0x07FF);
					ch[7]  = static_cast<int16_t>(	(buf[10] >> 5 | buf[11] << 3) & 0x07FF);
					ch[8]  = static_cast<int16_t>(	(buf[12]      | buf[13] << 8) & 0x07FF);
					ch[9]  = static_cast<int16_t>(	(buf[13] >> 3 | buf[14] << 5) & 0x07FF);
					ch[10] = static_cast<int16_t>(	(buf[14] >> 6 | buf[15] << 2  | buf[16] << 10) & 0x07FF);
					ch[11] = static_cast<int16_t>(	(buf[16] >> 1 | buf[17] << 7) & 0x07FF);
					ch[12] = static_cast<int16_t>(	(buf[17] >> 4 | buf[18] << 4) & 0x07FF);
					ch[13] = static_cast<int16_t>(	(buf[18] >> 7 | buf[19] << 1  | buf[20] << 9) & 0x07FF);
					ch[14] = static_cast<int16_t>(	(buf[20] >> 2 | buf[21] << 6) & 0x07FF);
					ch[15] = static_cast<int16_t>(	(buf[21] >> 5 | buf[22] << 3) & 0x07FF);
					
					ch17 = buf[23] & CH17_MASK;
					ch18 = buf[23] & CH18_MASK;
					lost_frame = buf[23] & LOST_FRAME_MASK;
					failsafe = buf[23] & FAILSAFE_MASK;
					if (failsafe==true)
						resetChannels();
					//Serial.print(" ch2, ch3= ");
					//Serial.print(int(ch[2]));
					//Serial.print(" ");
					//Serial.println(int(ch[3]));
					nrOfVoidComms=0;
				}
			}
			prevRxData = rxData;
		}
	}
}

bool sbus_protocol::build_cmd_to_motors_board()
{
	unsigned char temp;
	if (failsafe==true)
		temp=FAILSAFE_MASK;
	else
		temp=0;
	unsigned char cmd_buff[] ={
		highByte(ch[2]), lowByte(ch[2]), 
		highByte(ch[3]), lowByte(ch[3]), 
		highByte(ch[5]), lowByte(ch[5]),
		temp
	};
	//Serial.print(sizeof(cmd_buff));
	//pack the message whit header and footer, including escape bytes if necessary
	return pack_message_to_motors_board(cmd_buff, sizeof(cmd_buff));
}

bool sbus_protocol::pack_message_to_motors_board(unsigned char* msg_contents, int num_bytes)
{
	//Serial.printf("num_bytes=%d", num_bytes);
	send_buff_num_bytes=0;
	send_buff[send_buff_num_bytes++] = SBUS_TO_MOTORS_BOARD_PACKET_HEADER;

	// add contents
	for (int i=0; i<num_bytes; i++)
	{
		if (!add_byte_to_message(msg_contents[i], send_buff, sizeof(send_buff), send_buff_num_bytes))
			return false;
	}

	// add footer
	if (send_buff_num_bytes >= (sizeof(send_buff)-1))
	 return false;

	send_buff[send_buff_num_bytes++] = SBUS_TO_MOTORS_BOARD_PACKET_FOOTER;
	//Serial.printf(" send_buff_num_bytes=%d", send_buff_num_bytes);
	return true;
}

bool sbus_protocol::add_byte_to_message(unsigned char byte_to_add, unsigned char* msg_buff, int msg_buff_size, int16_t	&num_bytes)
{
	bool escape_required = (byte_to_add == SBUS_TO_MOTORS_BOARD_PACKET_HEADER ||
													byte_to_add == SBUS_TO_MOTORS_BOARD_PACKET_FOOTER ||
													byte_to_add == SBUS_TO_MOTORS_BOARD_PACKET_ESCAPE);

	// check if we have enough space
	if (num_bytes + (escape_required ? 2 : 1) >= msg_buff_size)
		return false;

	// add byte
	if (escape_required)
	{
		msg_buff[num_bytes++] = 0xAE;
		msg_buff[num_bytes++] = byte_to_add ^ SBUS_TO_MOTORS_BOARD_PACKET_ESCAPE_MASK;
	}
	else
		msg_buff[num_bytes++] = byte_to_add;

	return true;
}
