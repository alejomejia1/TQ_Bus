#define CRC8_H

#pragma once
#ifndef CRC8_H
#define CRC8_H

#include "Arduino.h"

class CRC8
{
	public:
	CRC8(){ }
		uint8_t crc8_maxim(const uint8_t *addr, uint8_t len);    
};
// -- END OF FILE --

#endif