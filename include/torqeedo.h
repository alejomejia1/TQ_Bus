#pragma once
#ifndef TORQEEDO_H
#define TORQEEDO_H

/*
 * Motor L:
 * TX -> 25
 * RX -> 26
 * RTS -> 14
 * 
 * Motor R
 * TX -> 27
 * RX -> 15
 * RTS -> 4
 * 
 * 
 */

#include "Arduino.h"

class TorqeedoMotor
{
  private:
    int _tx;
    int _rx;
    int _rts;
    int _onoff;

  public:
    TorqeedoMotor() { }
//    void begin(uint8_t ser,uint8_t tx, uint8_t rx, uint8_t rts, uint8_t onoff);
//    void tx();
//    void rx();
    
};

#endif