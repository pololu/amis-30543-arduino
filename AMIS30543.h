#pragma once

#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>

class AMIS30543Raw  // TODO: rename to AMIS30543SPI
{
public:

    // Addresses of control and status registers.  // TODO: move to AMIS30543
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

    enum stepMode
    {
        MicroStep128 = 128,
        MicroStep64 = 64,
        MicroStep32 = 32,
        MicroStep16 = 16,
        MicroStep8 = 8,
        MicroStep4 = 4,
        MicroStep2 = 2,
        MicroStep1 = 1,
        CompensatedHalf = MicroStep2,
        CompensatedFullTwoPhaseOn = MicroStep1,
        CompensatedFullOnePhaseOn = 200,
        UncompensatedHalf = 201,
        UncompensatedFull = 202,
    };

    enum statusFlag
    {
        OPENY = (1 << 0),
        OPENX = (1 << 1),
        WD = (1 << 2),
        CPFAIL = (1 << 3),
        TW = (1 << 4),
        OVCXNB = (1 << 5),
        OVCXNT = (1 << 6),
        OVCXPB = (1 << 7),
        OVCXPT = (1 << 8),
        TSD = (1 << 9),
        OVCYNB = (1 << 10),
        OVCYNT = (1 << 11),
        OVCYPB = (1 << 12),
        OVCYPT = (1 << 13),
    };

    void init(uint8_t slaveSelectPin)
    {
        driver.init(slaveSelectPin);
    }

    void resetSettings()
    {
        wr = cr0 = cr1 = cr2 = cr3 = 0;
        applySettings();
    }

    /*! Reads back all the SPI control registers from the device and
     * verifies that they are equal to the cached copies stored in this class.
     *
     * This can be used to verify that the driver is powered on and has not lost
     * them due to a power failure.  However this function will probably return
     * true if the driver is not powered and all of the cached settings are the
     * default values.  Therefore, we only recommend calling this after you have
     * changed at least one of the settings from its default value, for example
     * by calling enableDriver().
     *
     * @return 1 if the settings from the device match the cached copies, 0 if
     * they do not. */
    bool verifySettings()
    {
        return driver.readReg(AMIS30543Raw::WR) == wr &&
            driver.readReg(AMIS30543Raw::CR0) == cr0 &&
            driver.readReg(AMIS30543Raw::CR1) == cr1 &&
            driver.readReg(AMIS30543Raw::CR2) == cr2 &&
            driver.readReg(AMIS30543Raw::CR3) == cr3;
    }

    /*! Re-writes the cached settings stored in this class to the device.
     *
     * You should not normally need to call this function because settings are
     * written to the device whenever they are changed.  However, if
     * verifySettings() returns false (due to a power interruption, for
     * instance), then you could use applySettings to get the device's settings
     * back into the desired state. */
    void applySettings()
    {
        // Because of power interruption considerations, the register that
        // contains the MOTEN bit (CR2) must be written first, and whenever we
        // write to it we should also write to all the other registers.

        // CR2 is written first, because it contains the MOTEN bit, and there is
        // a risk that there might be a power interruption to the driver right
        // before CR2 is written.  This could result in the motor being enabled
        // with incorrect settings.  Also, whenever we do write to CR2, we want to
        // also write the other registers to make sure they are in the correct state.
        driver.writeReg(AMIS30543Raw::CR2, cr2);

        writeWR();
        writeCR0();
        writeCR1();
        writeCR3();
    }

    void enableDriver()
    {
        cr2 |= 0b10000000;
        applySettings();
    }

    void disableDriver()
    {
        cr2 &= ~0b10000000;
        applySettings();
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
     *
     * Our tests indicated that the return value of this function might be wrong
     * if you read it within 25 microseconds of commanding the driver to take a
     * step.  Therefore we recommend delaying for at least 100 microseconds after
     * taking a step and before calling this function. */
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

    /*! Configures the driver to have the specified stepping mode.
     *
     * This affects many things about the performance of the motor, including
     * how much the output moves for each step taken and how much current flows
     * through the coils in each stepping position.
     *
     * If an invalid stepping mode is passed to this function, then it selects
     * 1/32 micro-step, which is the driver's default. */
    void setStepMode(uint8_t mode)
    {
        // Pick 1/32 micro-step by default.
        uint8_t esm = 0b000;
        uint8_t sm = 0b000;

        // The order of these cases matches the order in Table 12 of the
        // AMIS-30543 datasheet.
        switch(mode)
        {
        case MicroStep32: sm = 0b000; break;
        case MicroStep16: sm = 0b001; break;
        case MicroStep8: sm = 0b010; break;
        case MicroStep4: sm = 0b011; break;
        case CompensatedHalf: sm = 0b100; break; /* a.k.a. MicroStep2 */
        case UncompensatedHalf: sm = 0b101; break;
        case UncompensatedFull: sm = 0b110; break;
        case MicroStep128: esm = 0b001; break;
        case MicroStep64: esm = 0b010; break;
        case CompensatedFullTwoPhaseOn: esm = 0b011; break;  /* a.k.a. MicroStep 1 */
        case CompensatedFullOnePhaseOn: esm = 0b100; break;
        }

        cr0 = (cr0 & ~0b11100000) | (sm << 5);
        cr3 = (cr3 & ~0b111) | esm;
        writeCR0();
        writeCR3();
    }

    /*! Sets the SLP bit 1, enabling sleep mode.
     *
     * According to the AMIS-30543 datasheet, the motor supply voltage must be
     * at least 9 V before entering sleep mode.
     *
     * You can call sleepStop() to disable sleep mode.
     *
     * Please read the note about this function in README.md. */
    void sleep()
    {
        cr2 |= (1 << 6);
        applySettings();
    }

    /*! Sets the SLP bit 0, disabling sleep mode.
     *
     * Please read the note about this function in README.md. */
    void sleepStop()
    {
        cr2 &= ~(1 << 6);
        applySettings();
    }

    /*! Sets the value of the NXTP configuration bit to 0, which means that new
     * steps are triggered by a rising edge on the NXT/STEP pin.  This is the
     * default behavior. */
    void stepOnRisingEdge()
    {
        cr1 &= ~0b01000000;
        writeCR1();
    }

    /*! Sets the value of the NXTP configuration bit to 1, which means that new
     * steps are triggered by a falling edge on the NXT/STEP pin. */
    void stepOnFallingEdge()
    {
        cr1 |= 0b01000000;
        writeCR1();
    }

    /*! Sets the PWMF bit to 1, which doubles the PWM frequency (45.6 kHz) .*/
    void setPwmFrequencyDouble()
    {
        cr1 |= (1 << 3);
        writeCR1();
    }

    /*! Clears the PWMF bit, which sets the PWM frequency to its default value
     *  (22.8 kHz). */
    void setPwmFrequencyDefault()
    {
        cr1 &= ~(1 << 3);
        writeCR1();
    }

    /*! Sets the PWMJ bit, which enables artificial jittering in the PWM signal
     *  used to control the current to each coil. */
    void setPwmJitterOn()
    {
        cr1 |= (1 << 2);
        writeCR1();
    }

    /*! Clears the PWMJ bit, which disables artificial jittering in the PWM
     *  signal used to control the current to each coil.  This is the default
     *  setting. */
    void setPwmJitterOff()
    {
        cr1 &= ~(1 << 2);
        writeCR1();
    }

    /*! This sets the EMC[1:0] bits, which determine how long it takes the PWM
     *  signal to rise or fall.  Valid values are 0 through 3.  Higher values
     *  correspond to longer rise and fall times.  **/
    void setPwmSlope(uint8_t emc)
    {
        cr1 = (cr1 & ~0b11) | (emc & 0b11);
        writeCR1();
    }

    /*! Clears the SLAG bit, which configures the signal on SLA pin to have a
     *  gain of 0.5 (the default).
     *
     * Please read the note about this function in README.md. */
    void setSlaGainDefault()
    {
        cr2 &= ~(1 << 5);
        applySettings();
    }

    /*! Sets the SLAG bit to 1, which configures the signal on SLA pin to have a
     *  gain of 0.25 (half of the default).
     *
     * Please read the note about this function in README.md. */
    void setSlaGainHalf()
    {
        cr2 |= (1 << 5);
        applySettings();
    }

    /*! Set the SLAT bit to 0 (the default), which disables transparency on the
     *  SLA pin.  See the AMIS-30543 datasheet for more information.
     *
     * Please read the note about this function in README.md. */
    void setSlaTransparencyOff()
    {
        cr2 &= ~(1 << 4);
        applySettings();
    }

    /*! Sets the SLAT bit to 1, which enables transparency on the SLA pin.
     *  See the AMIS-30543 datasheet for more information.
     *
     * Please read the note about this function in README.md. */
    void setSlaTransparencyOn()
    {
        cr2 |= (1 << 4);
        applySettings();
    }

    /*! Reads the status flags from registers SR0, SR1, and SR2.
     *
     * The return value is a 16-bit unsigned integer that has one bit for each
     * status flag.  You can simply compare the return value to 0 to see if any
     * of the status flags are set, or you can use the logical and operator (`&`)
     * and the statusFlag enum to check individual flags.
     *
     * ~~~~{.cpp}
     * uint16_t flags = stepper.readStatusFlags();
     * if (flags)
     * {
     *   // At least one flag is set.
     *
     *   if (flags & AMIS3054::TSD)
     *   {
     *     // Thermal shutdown flag is set.
     *   }
     *
     * }
     * ~~~~
     */
    uint16_t readStatusFlags()
    {
        uint8_t sr0 = readStatusReg(AMIS30543Raw::SR0);
        uint8_t sr1 = readStatusReg(AMIS30543Raw::SR1);
        uint8_t sr2 = readStatusReg(AMIS30543Raw::SR2);

        return ((sr0 & 0x7C) >> 2) | ((sr1 & 0x78) << 2) | ((sr2 & 0x7C) << 7);
    }

protected:

    uint8_t wr, cr0, cr1, cr2, cr3;

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

    void writeCR3()
    {
        driver.writeReg(AMIS30543Raw::CR3, cr3);
    }

public:
    // This is only marked as public for the purpose of testing; you should not
    // use it normally.
    AMIS30543Raw driver;
};
