#pragma once
#ifndef REMOTE_H
#define REMOTE_H


#include "Arduino.h"

class RemoteSignal {

    private:
        unsigned long _pulseInTimeBegin;
        unsigned long _pulseInTimeEnd;
        bool newPulseDurationAvailable = false;

    void begin();
    void buttonPinInterrupt();
}