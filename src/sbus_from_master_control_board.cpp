#include "sbus_from_master_control_board.h"

#include "HardwareSerial.h"

int nrOfVoidSbusComms=10;
short sBusCh[3]={1072, 1072, 1072};
bool sBusFailsafe=false;
bool sBusJustUpdated=false;

unsigned char sBusRxFrame[256];
int sBusRxFrameLen=0;

void parse_sbus_from_master_packet()
{
	unsigned char realRxFrame[256];
	int realRxFrameLen=0;
	for (int i=0; i<sBusRxFrameLen; i++)
	{
		if (sBusRxFrame[i]==SBUS_FROM_MASTER_PACKET_ESCAPE)
		{
			i++;	//escape bayte is not to be used; the data byte to use is the next byte in the frame
			realRxFrame[realRxFrameLen]=sBusRxFrame[i] ^ SBUS_FROM_MASTER_PACKET_ESCAPE_MASK;
		}
		else
			realRxFrame[realRxFrameLen]=sBusRxFrame[i];
		realRxFrameLen++;
	}

	if (realRxFrameLen==7)
	{
		sBusCh[0]  = static_cast<int16_t>(realRxFrame[0]<<8 | realRxFrame[1]);
		sBusCh[1]  = static_cast<int16_t>(realRxFrame[2]<<8 | realRxFrame[3]);
		sBusCh[2]  = static_cast<int16_t>(realRxFrame[4]<<8 | realRxFrame[5]);
		sBusFailsafe = realRxFrame[6] & SBUS_FROM_MASTER_FAILSAFE_MASK;
		//Serial.println(" sbus protocol from master complied");
		//Serial.printf("sBusCh[0]= %d, sBusCh[1]= %d, sBusCh[2]= %d ", sBusCh[0], sBusCh[1], sBusCh[2]);
		//Serial.printf("sBusFailsafe= %d\n", sBusFailsafe);
		nrOfVoidSbusComms=0;
		sBusJustUpdated=true;
	}
}

void decode_sbus_from_master_info()
{
	static bool headerByteWasReceived=false;
	int bytesToRead;
	unsigned char rxData;
	
	bytesToRead=Serial.available();
	if (bytesToRead)
	{
		//Serial.print("Sbus info from master= ");
		for (int i=0; i<bytesToRead; i++)
		{
			rxData=Serial.read();
			//Serial.printf(" 0x%02x",rxData);
			if (rxData==SBUS_FROM_MASTER_PACKET_HEADER)
			{
				sBusRxFrameLen=0;
				headerByteWasReceived=true;
			}
			else if (rxData==SBUS_FROM_MASTER_PACKET_FOOTER)
			{
				if (headerByteWasReceived==true)
				{
					parse_sbus_from_master_packet();
					sBusRxFrameLen=0;
					headerByteWasReceived=false;
				}
			}
			else
				sBusRxFrame[sBusRxFrameLen++]=rxData;
		}
	}

}

