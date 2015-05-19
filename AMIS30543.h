#pragma once

#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>

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

    uint8_t readReg(uint8_t address)
    {
        selectChip();
        SPI.transfer(address);
        uint8_t dataOut = SPI.transfer(0);
        deselectChip();
        return dataOut;
    }

    void writeReg(uint8_t address, uint8_t value)
    {
        selectChip();
        SPI.transfer(address);
        SPI.transfer(value);
        deselectChip();
    }

private:

    void selectChip()
    {
        digitalWrite(ssPin, LOW);
    }

    void deselectChip()
    {
       digitalWrite(ssPin, HIGH);

       // The CS high time is specified as 2.5 us in the
       // AMIS-30543 datasheet.
       delayMicroseconds(3);
    }

    uint8_t ssPin;
};
