#include "torqeedo_motor_control.h"

void torqeedo_motor_control::beginMotorSerial(uint8_t ser, uint8_t rxPin, uint8_t txPin, uint8_t onOffPin)
{
	if (ser==1 || ser==2)
	{ 
		_ser = ser;
		_txPin = txPin;
		_rxPin = rxPin;
		_onOffPin = onOffPin;
		
		pinMode(_onOffPin, OUTPUT);

		if(_ser==1) 
		{
			Serial1.begin(19200, SERIAL_8N1, rxPin, txPin);
		} 
		else 
		{
			Serial2.begin(19200, SERIAL_8N1, rxPin, txPin);
		}
	}
}

void torqeedo_motor_control::on()
{
	digitalWrite(_onOffPin, HIGH);
	delay(500); // Wait 500 ms
	digitalWrite(_onOffPin, LOW);
}

void torqeedo_motor_control::off()
{
	digitalWrite(_onOffPin, HIGH);
	delay(6000);
	digitalWrite(_onOffPin, LOW);
}

void torqeedo_motor_control::initVariables()
{
	nrOfvoidComms=10;
	motorDummyOrderCount=0;
	restartDelayTxTimer=false;
	txFlag=false;
	sBusCommsOk=false;
	up_down_sbus_throttle_val=1072;
	left_right_sbus_throttle_val=1072;
	arm_button_val=172;

	_motor_speed_desired=0;
	_motor_user_value=-100;
	_throttle_user_value=0;
	_send_buff_num_bytes=0;
	_rxFrameLen=0;
	_headerByteWasReceived=false;

	//_lastReqToMotorsMillis=millis();
} 

void torqeedo_motor_control::frameTransmission_slot(int motorId)
{
	bool packetizing_is_ok;
	//Serial.println(motorDummyOrderCount);
	if (motorDummyOrderCount < NR_OF_REPETITIVE_INITIAL_ORDER_TO_MOTOR)
	{
		_motor_speed_desired=-100;
		packetizing_is_ok = build_motor_speed_cmd(motorId);
		motorDummyOrderCount++;
	}
	else if (motorDummyOrderCount < (NR_OF_REPETITIVE_INITIAL_ORDER_TO_MOTOR + NR_OF_REPETITIVE_INITIAL_ZERO_ORDER_TO_MOTOR))
	{
		_motor_speed_desired=0;
		packetizing_is_ok = build_motor_speed_cmd(motorId);
		motorDummyOrderCount++;
	}
	else
	{
		if (false)
			_motor_speed_desired=_motor_user_value;
		else
		{
			if (sBusCommsOk==false)
				_throttle_user_value=0;
			else if (arm_button_val<800)
				_throttle_user_value=0;
			else
			{
				short up_down_throttle=mapThrottleVar(up_down_sbus_throttle_val);
				short left_right_throttle=mapThrottleVar(left_right_sbus_throttle_val);
				if (motorId==0)
					_throttle_user_value = up_down_throttle + left_right_throttle;
				else
					_throttle_user_value = up_down_throttle - left_right_throttle;
				//Serial.print("_throttle_user_value=");
				//Serial.println(_throttle_user_value);
			}
			_motor_speed_desired=_throttle_user_value;
		}
		packetizing_is_ok = build_motor_speed_cmd(motorId);
	}

	if (packetizing_is_ok==true && _send_buff_num_bytes>0)
	{
		for (int i=0; i<_send_buff_num_bytes; i++)
		{
			if (motorId==0)
				Serial1.write(_send_buff[i]);
			else
				Serial2.write(_send_buff[i]);
		}
		//QDebug deb=qDebug().noquote();
		//deb<<"send_buff motor"<<motId<<"=";
		//for (int i=0; i<ba.count(); i++)
		//	deb<<QString("0x%1").arg((unsigned char)ba.at(i), 2, 16, QLatin1Char('0'));
	}
	txFlag=false;
}

bool torqeedo_motor_control::build_motor_speed_cmd(int motorId)
{
	unsigned char mot_speed_cmd_buff[] = {BUS_MASTER, 0x00, 0x05, 0x00, highByte(_motor_speed_desired), lowByte(_motor_speed_desired)};
	/*
	Serial.printf(" M%d", motorId);
	for (int i=0; i<sizeof(mot_speed_cmd_buff); i++)
	{
		Serial.print(" 0x");
		Serial.print(mot_speed_cmd_buff[i], HEX);
	}
	Serial.println("");
	*/

	//pack the message whit header, crc and footer, including escape bytes if necessary
	return pack_message(mot_speed_cmd_buff, sizeof(mot_speed_cmd_buff));
}

bool torqeedo_motor_control::pack_message(unsigned char* msg_contents, int num_bytes)
{
	//qDebug()<<"num_bytes="<<num_bytes;
	//qDebug()<<"sizeof(send_buff)="<<sizeof(send_buff);
	// calculate crc
	unsigned char crc = crc8_maxim(msg_contents, num_bytes);

	_send_buff_num_bytes=0;
	// add header
	_send_buff[_send_buff_num_bytes++] = TORQEEDO_PACKET_HEADER;

	// add contents
	for (int i=0; i<num_bytes; i++)
	{
		if (!add_byte_to_message(msg_contents[i], _send_buff, sizeof(_send_buff), _send_buff_num_bytes))
			return false;
	}

	// add crc
	if (!add_byte_to_message(crc, _send_buff, sizeof(_send_buff), _send_buff_num_bytes))
	 return false;

	// add footer
	if (_send_buff_num_bytes >= (sizeof(_send_buff)-2))
	 return false;

	_send_buff[_send_buff_num_bytes++] = TORQEEDO_PACKET_FOOTER;
	_send_buff[_send_buff_num_bytes++] = 0xff;
	return true;
}

unsigned char torqeedo_motor_control::crc8_maxim(unsigned char * mot_speed_cmd_buff, int mot_speed_cmd_buff_len)
{
	unsigned char crc = 0x00;
	int i=0;
	/*
	QDebug deb=qDebug().noquote();
	deb<<"mot_speed_cmd_buff=";
	for (int i=0; i< mot_speed_cmd_buff_len; i++)
	{
		deb<<QString("0x%1").arg(mot_speed_cmd_buff[i], 2, 16, QLatin1Char('0'));
	}
	*/

	while (mot_speed_cmd_buff_len>0)
	{
		crc = CRC8_TABLE_MAXIM[crc ^ mot_speed_cmd_buff[i++]];
		mot_speed_cmd_buff_len--;
	}
	return crc;
}

bool torqeedo_motor_control::add_byte_to_message(unsigned char byte_to_add, unsigned char* msg_buff, int msg_buff_size, int	&num_bytes)
{
	bool escape_required = (byte_to_add == TORQEEDO_PACKET_HEADER ||
													byte_to_add == TORQEEDO_PACKET_FOOTER ||
													byte_to_add == TORQEEDO_PACKET_ESCAPE);

	// check if we have enough space
	if (num_bytes + (escape_required ? 2 : 1) >= msg_buff_size)
		return false;

	// add byte
	if (escape_required)
	{
		msg_buff[num_bytes++] = TORQEEDO_PACKET_ESCAPE;
		msg_buff[num_bytes++] = byte_to_add ^ TORQEEDO_PACKET_ESCAPE_MASK;
	}
	else
		msg_buff[num_bytes++] = byte_to_add;

	return true;
}

short torqeedo_motor_control::mapThrottleVar(unsigned short value)
{
	static const int THROTTLE_MAX=1972;
	static const int THROTTLE_MIN=172;
	static const int THROTTLE_GAP=THROTTLE_MAX-THROTTLE_MIN;
	static const int MOTOR_MAX=1000;
	static const int MOTOR_MIN=-1000;
	static const int MOTOR_GAP=MOTOR_MAX-MOTOR_MIN;

	if (value < THROTTLE_MIN)
		value = THROTTLE_MIN;
	else if (value > THROTTLE_MAX)
		value = THROTTLE_MAX;
	float factor = ((float)(value-THROTTLE_MIN))/THROTTLE_GAP;
	float returnValue=factor*MOTOR_GAP;
	returnValue+=MOTOR_MIN;
	//qDebug()<<"value="<<value<<"ret value="<<returnValue;
	return returnValue;
}

void torqeedo_motor_control::msgReceived_slot(int connId)
{
	unsigned char rxBuff[256];
	unsigned char rxData;
	int bytesToRead;
	if (connId==0)
	{
		bytesToRead=Serial1.available();
		if (bytesToRead)
		{
			for (int i=0; i<bytesToRead; i++)
				rxBuff[i]=Serial1.read();
		}
	}
	else
	{
		bytesToRead=Serial2.available();
		if (bytesToRead)
		{
			for (int i=0; i<bytesToRead; i++)
				rxBuff[i]=Serial2.read();
		}
	}

	if (bytesToRead)
	{
		/*
		Serial.print("m");
		Serial.print(connId);
		Serial.print(" len=");
		Serial.print(bytesToRead);
		Serial.println("");
		*/
	
		for (int i=0; i<bytesToRead; i++)
		{
			rxData=rxBuff[i];
			if (rxData==TORQEEDO_PACKET_HEADER)
			{
				#ifdef DEBUB_MOTORS_INFO
				Serial.println(" AC_rx");
				#endif
				_rxFrameLen=0;
				_headerByteWasReceived=true;
			}
			else if (rxData==TORQEEDO_PACKET_FOOTER)
			{
				#ifdef DEBUB_MOTORS_INFO
				Serial.print(" AD_rx");
				#endif
				if (_headerByteWasReceived==true)
				{
					parse_packet(connId);
					_rxFrameLen=0;
					_headerByteWasReceived=false;
				}
			}
			else
				_rxFrame[_rxFrameLen++]=rxData;
		}
	}
	nrOfvoidComms=0;
}

void torqeedo_motor_control::parse_packet(int connId)
{
	unsigned char realRxFrame[256];
	int realRxFrameLen=0;
	for (int i=0; i<_rxFrameLen; i++)
	{
		if (_rxFrame[i]==TORQEEDO_PACKET_ESCAPE)
		{
			i++;	//escape bayte is not to be used; the data byte to use is the next byte in the frame
			realRxFrame[realRxFrameLen]=_rxFrame[i] ^ TORQEEDO_PACKET_ESCAPE_MASK;
		}
		else
			realRxFrame[realRxFrameLen]=_rxFrame[i];
		realRxFrameLen++;
	}

	bool unknowFlag=false;
	switch(realRxFrameLen)
	{
		case 3:
			if (realRxFrame[0]==REMOTE_MESSAGE_ID && realRxFrame[1]==0x01 && realRxFrame[2]==0x89)
			{
				//#ifdef DEBUB_MOTORS_INFO
				//Serial.print("	0-REQ_SPEED_FROM_BATT_TO_REMOTE Len=");
				//Serial.println(realRxFrameLen);

				/*
				int32_t now=millis();
				int32_t diff=now - _lastReqToMotorsMillis;
				_lastReqToMotorsMillis=millis();
				Serial.printf("time from last mot req=%4d ", diff);
				*/
				//#endif
				restartDelayTxTimer=true;
			}
			else if (realRxFrame[0]==0x30 && realRxFrame[1]==0x01 && realRxFrame[2]==0x73)
			{
				#ifdef DEBUB_MOTORS_INFO
				Serial.print("	2-REQ_MOTOR_STATUS_FROM_BATT_TO_MOTOR Len=");
				Serial.println(realRxFrameLen);
				#endif
			}
			else if (realRxFrame[0]==0x30 && realRxFrame[1]==0x03 && realRxFrame[2]==0xCF)
			{
				#ifdef DEBUB_MOTORS_INFO
				Serial.print("	4-REQ_MOTOR_PARAMETERS_FROM_BATT_TO_MOTOR Len=");
				Serial.println(realRxFrameLen);
				#endif
			}
			else if (realRxFrame[0]==0x00)
			{
				#ifdef DEBUB_MOTORS_INFO
				Serial.print("	7-SEND_MOTOR_POWER_ORDER_ACK_FROM_MOTOR_TO_BATT Len=");
				Serial.println(realRxFrameLen);
				#endif
			}
			else
				unknowFlag=true;
		break;

		case 6:
			if (realRxFrame[0]==0x00)
			{
				#ifdef DEBUB_MOTORS_INFO
				Serial.print("	3-SEND_MOTOR_STATUS_FROM_MOTOR_TO_BATT Len=");
				Serial.println(realRxFrameLen);
				#endif
			}
			else
				unknowFlag=true;
		break;

		case 7:
			if (realRxFrame[0]==0x30)
			{
				#ifdef DEBUB_MOTORS_INFO
				Serial.print("	6-SEND_MOTOR_POWER_ORDER_FROM_BATT_TO_MOTOR Len=");
				Serial.println(realRxFrameLen);
				#endif
			}
			else
				unknowFlag=true;
		break;

		case 13:
			if (realRxFrame[0]==LCD_MESSAGE_ID && realRxFrame[1]==SYSTEM_SETUP)
			{
				#ifdef DEBUB_MOTORS_INFO
				Serial.print("	9-SEND_SYSTEM_SETUP_FROM_LCD_TO_BATT Len=");
				Serial.println(realRxFrameLen);
				#endif
				display_system_setup.motor_type = realRxFrame[3];
				display_system_setup.motor_sw_version = (realRxFrame[4]<<8 | realRxFrame[5]);
				display_system_setup.batt_capacity = (realRxFrame[6]<<8 | realRxFrame[7]);
				display_system_setup.batt_charge_pct = realRxFrame[8];
				display_system_setup.batt_type = realRxFrame[9];
				display_system_setup.master_sw_version =  (realRxFrame[10]<<8 | realRxFrame[11]);
				//qDebug()<<"batt_capacity="<<display_system_setup[connId].batt_capacity;
				//qDebug()<<"batt_charge_pct="<<display_system_setup[connId].batt_charge_pct;
			}
			else
				unknowFlag=true;
		break;

		case 15:
			if (realRxFrame[0]==0x00)
			{
				#ifdef DEBUB_MOTORS_INFO
				Serial.print("	5-SEND_MOTOR_PARAMETERS_FROM_MOTOR_TO_BATT Len=");
				Serial.println(realRxFrameLen);
				#endif
			}
			else
				unknowFlag=true;
		break;

		case 30:
			if (realRxFrame[0]==LCD_MESSAGE_ID && realRxFrame[1]==SYSTEM_STATE)
			{
				#ifdef DEBUB_MOTORS_INFO
				Serial.print("	8-SEND_SYSTEM_STATE_FROM_LCD_TO_BATT Len=");
				Serial.println(realRxFrameLen);
				#endif
				display_system_state.flags = (realRxFrame[2]<<8 | realRxFrame[3]);
				display_system_state.master_state = realRxFrame[4]; // deprecated
				display_system_state.master_error_code = realRxFrame[5];
				display_system_state.motor_voltage = (realRxFrame[6]<<8 | realRxFrame[7]) * 0.01;
				display_system_state.motor_current = (realRxFrame[8]<<8 |realRxFrame[9]) * 0.1;
				display_system_state.motor_power = (realRxFrame[10]<<8 | realRxFrame[11]);
				display_system_state.motor_rpm = (short)(realRxFrame[12]<<8 | realRxFrame[13]);
				display_system_state.motor_pcb_temp = realRxFrame[14];
				display_system_state.motor_stator_temp = realRxFrame[15];
				display_system_state.batt_charge_pct = realRxFrame[16];
				display_system_state.batt_voltage = (realRxFrame[17]<<8 | realRxFrame[18]) * 0.01;
				display_system_state.batt_current = (realRxFrame[19]<<8 |realRxFrame[20]) * 0.1;
				display_system_state.gps_speed = (realRxFrame[21]<<8 | realRxFrame[22]);
				display_system_state.range_miles = (realRxFrame[23]<<8 | realRxFrame[24]);
				display_system_state.range_minutes = (realRxFrame[25]<<8 | realRxFrame[26]);
				display_system_state.temp_sw = realRxFrame[27];
				display_system_state.temp_rp = realRxFrame[28];
				
				#ifdef DEBUB_MOTORS_INFO
				//qDebug()<<QString("flags= 0x%1").arg(_display_system_state[connId].flags, 4, 16, QLatin1Char('0'));
				Serial.print("batt_charge_pct=");	Serial.println(display_system_state.batt_charge_pct);
				Serial.print("motor_voltage=");		Serial.println(display_system_state.motor_voltage);
				//Serial.print("motor_current=");		Serial.println(display_system_state.motor_current);
				//Serial.print("motor_power=");			Serial.println(display_system_state.motor_power);
				Serial.print("motor_rpm=");				Serial.println(display_system_state.motor_rpm);
				#endif
			}
			else
				unknowFlag=true;
		break;

		default:
				unknowFlag=true;
			break;
	}
	if (unknowFlag)
	{
		#ifdef DEBUB_MOTORS_INFO
		Serial.println("	UNKNOW Len=");
		Serial.println(realRxFrameLen);
		#endif
		/*
		QDebug deb=qDebug().noquote();
		deb<<"	UNKNOW."<<"Len="<<realRxFrameLen;
		deb<<"realRxFrame=";
		for (int i=0; i<realRxFrameLen; i++)
			deb<<QString("0x%1").arg(realRxFrame.at(i), 2, 16, QLatin1Char('0'));
		*/
	}
}




