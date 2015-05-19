#pragma once

#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>

class AMIS30543Raw
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
        transfer(address);
        uint8_t dataOut = transfer(0);
        deselectChip();
        return dataOut;
    }

    void writeReg(uint8_t address, uint8_t value)
    {
        selectChip();
        transfer(address);
        transfer(value);

        // The CS line must go high after writing for the value to
        // actually take effect.
        deselectChip();
    }

private:

    SPISettings settings = SPISettings(500000, MSBFIRST, SPI_MODE0);

    uint8_t transfer(uint8_t value)
    {
        return SPI.transfer(value);
    }

    void selectChip()
    {
        digitalWrite(ssPin, LOW);
        SPI.beginTransaction(settings);
    }

    void deselectChip()
    {
       digitalWrite(ssPin, HIGH);
       SPI.endTransaction();

       // The CS high time is specified as 2.5 us in the
       // AMIS-30543 datasheet.
       delayMicroseconds(3);
    }

    uint8_t ssPin;
};

class AMIS30543
{
public:

    void init(uint8_t slaveSelectPin)
    {
        driver.init(slaveSelectPin);
        wr = cr0 = cr1 = cr2 = cr3 = 0;
        applySettings();
    }

    void applySettings()
    {
        writeWR();
        writeCR0();
        writeCR1();
        writeCR2();
        writeCR3();
    }

    void enableDriver()
    {
        cr0 = 10; // tmphax
        writeCR0(); // tmphax

        cr2 |= 0b10000000;
        writeCR2();

    }

    void disableDriver()
    {
        cr2 &= ~0b10000000;
        writeCR2();
    }

private:
    uint8_t wr;
    uint8_t cr0;
    uint8_t cr1;
    uint8_t cr2;
    uint8_t cr3;

    // Addresses of control and status registers.
    enum regAddr
    {
        WR  = 0x0,
        CR0 = 0x1,
        CR1 = 0x2,
        CR2 = 0x3,
        CR3 = 0x9,
        SR0 = 0x4,
        SR1 = 0x5,
        SR2 = 0x6,
        SR3 = 0x7,
        SR4 = 0xA,
    };

    void writeWR()
    {
        driver.writeReg(WR, wr);
    }

    void writeCR0()
    {
        driver.writeReg(CR0, cr0);
    }

    void writeCR1()
    {
        driver.writeReg(CR1, cr1);
    }

    void writeCR2()
    {
        driver.writeReg(CR2, cr2);
    }

    void writeCR3()
    {
        driver.writeReg(CR3, wr);
    }

    AMIS30543Raw driver;
};
