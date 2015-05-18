#pragma once

#include <stdint.h>
#include <Arduino.h>

struct AMIS30543Settings
{
    uint8_t WR;
    uint8_t CR0;
    uint8_t CR1;
    uint8_t CR2;
    uint8_t CR3;

    AMIS30543Settings()
    {
        WR = CR0 = CR1 = CR2 = CR3 = 0;
    }
};

class AMIS30543
{
public:

    void init(uint8_t slaveSelectPin)
    {
        ssPin = slaveSelectPin;

        digitalWrite(slaveSelectPin, HIGH);
        pinMode(slaveSelectPin, OUTPUT);
    }

private:

    uint8_t ssPin;
};
