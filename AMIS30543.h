#pragma once

#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>

class AMIS30543Raw
{
public:

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

    void init(uint8_t slaveSelectPin)
    {
        ssPin = slaveSelectPin;

        digitalWrite(ssPin, HIGH);
        pinMode(ssPin, OUTPUT);
    }

    uint8_t readReg(uint8_t address)
    {
        selectChip();
        transfer(address & 0b11111);
        uint8_t dataOut = transfer(0);
        deselectChip();
        return dataOut;
    }

    void writeReg(uint8_t address, uint8_t value)
    {
        selectChip();
        transfer(0x80 | (address & 0b11111));
        transfer(value);

        // The CS line must go high after writing for the value to actually take
        // effect.
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

       // The CS high time is specified as 2.5 us in the AMIS-30543 datasheet.
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
    }

    void resetSettings()
    {
        wr = cr0 = cr1 = cr2 = cr3 = 0;
        applySettings();
    }

    void enableDriver()
    {
        cr2 |= 0b10000000;
        writeCR2();
    }

    void disableDriver()
    {
        cr2 &= ~0b10000000;
        writeCR2();
    }

    /*! Sets the per-coil current limit equal to the highest available setting
     * that is less than the given current, in units of milliamps.
     *
     * Refer to Table 13 of the AMIS 30543 datasheet to see which current limits
     * are available. */
    void setCurrentMilliamps(uint16_t current)
    {
        // This comes from Table 13 of the AMIS-30543 datasheet.
        uint8_t code = 0;
        if      (current >= 3000) { code = 0b11001; }
        else if (current >= 2845) { code = 0b11000; }
        else if (current >= 2700) { code = 0b10111; }
        else if (current >= 2440) { code = 0b10110; }
        else if (current >= 2240) { code = 0b10101; }
        else if (current >= 2070) { code = 0b10100; }
        else if (current >= 1850) { code = 0b10011; }
        else if (current >= 1695) { code = 0b10010; }
        else if (current >= 1520) { code = 0b10001; }
        else if (current >= 1405) { code = 0b10000; }
        else if (current >= 1260) { code = 0b01111; }
        else if (current >= 1150) { code = 0b01110; }
        else if (current >= 1060) { code = 0b01101; }
        else if (current >=  955) { code = 0b01100; }
        else if (current >=  870) { code = 0b01011; }
        else if (current >=  780) { code = 0b01010; }
        else if (current >=  715) { code = 0b01001; }
        else if (current >=  640) { code = 0b01000; }
        else if (current >=  585) { code = 0b00111; }
        else if (current >=  540) { code = 0b00110; }
        else if (current >=  485) { code = 0b00101; }
        else if (current >=  445) { code = 0b00100; }
        else if (current >=  395) { code = 0b00011; }
        else if (current >=  355) { code = 0b00010; }
        else if (current >=  245) { code = 0b00001; }

        cr0 = (cr0 & 0b11100000) | code;
        writeCR0();
    }

    /*! Reads the current microstepping position, which is a number between 0
     * and 511.
     *
     * The different positions and their corresponding coil values are listed in
     * Table 9 of the AMIS 30543 datasheet.
     *
     * The lower two bits of this return value might be inaccurate if the step
     * pin is being toggled while this function runs (e.g. from an interrupt or
     * a PWM signal).
     */
    uint16_t readPosition()
    {
        uint8_t sr3 = readStatusReg(AMIS30543Raw::SR3);
        uint8_t sr4 = readStatusReg(AMIS30543Raw::SR4);
        return ((uint16_t)sr3 << 2) | (sr4 & 3);
    }

    /*! Sets the value of the DIRCTRL configuration bit.
     *
     * Allowed values are 0 or 1.
     *
     * You can use this command to control the direction of the stepper motor
     * and simply leave the DIR pin disconnected. */
    void setDirection(bool value)
    {
        if (value)
        {
            cr1 |= 0x80;
        }
        else
        {
            cr1 &= ~0x80;
        }
        writeCR1();
    }

protected:
    uint8_t wr;
    uint8_t cr0;
    uint8_t cr1;
    uint8_t cr2;
    uint8_t cr3;

    void applySettings()
    {
        writeWR();
        writeCR0();
        writeCR1();
        writeCR2();
        writeCR3();
    }

    uint8_t readStatusReg(uint8_t address)
    {
        // Mask off the parity bit.
        // (Later we might add code here to check the parity
        // bit and record errors.)
        return driver.readReg(address) & 0x7F;
    }

    void writeWR()
    {
        driver.writeReg(AMIS30543Raw::WR, wr);
    }

    void writeCR0()
    {
        driver.writeReg(AMIS30543Raw::CR0, cr0);
    }

    void writeCR1()
    {
        driver.writeReg(AMIS30543Raw::CR1, cr1);
    }

    void writeCR2()
    {
        driver.writeReg(AMIS30543Raw::CR2, cr2);
    }

    void writeCR3()
    {
        driver.writeReg(AMIS30543Raw::CR3, cr3);
    }

public:
    // This is only marked as public for the purpose of testing; you should not
    // use it normally.
    AMIS30543Raw driver;
};
