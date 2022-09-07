#define CRC8_H

#ifndef CRC8_H
#define CRC8_H

#include <Arduino.h>
#include "crc8.h"

uint8_t CRC8::crc8_maxim(const uint8_t *addr, uint8_t len)
{
  uint8_t crc = 0;
  static const uint8_t PROGMEM dscrc2x16_table[] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
    0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
  };
  while (len--) {
    crc = *addr++ ^ crc;  // just re-using crc as intermediate
    crc = pgm_read_byte(dscrc2x16_table + (crc & 0x0f)) ^
    pgm_read_byte(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
  }

  return crc;
}

#endif
