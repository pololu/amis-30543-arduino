// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/

/*! \file AMIS30543.h
 *
 * This is the main header file for the AMIS30543 library, a library for
 * controllering the AMIS-30543 micro-stepping stepper motor driver.
 *
 * For an overview of the features of this library, see
 *
 *   https://github.com/pololu/fastgpio-arduino
 *
 * That is the main repository for this library,
 *
 */

#pragma once

#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>

/*! This class provides low-level functions for reading and writing from the SPI
 * interface of an AMIS-30543 micro-stepping stepper motor driver.
 *
 * Most users should use the AMIS30543 class, which provides a higher-level
 * interface, instead of this class. */
class AMIS30543SPI
{
public:

    /*! Configures this object to use the specified pin as a slave select pin.
     * You must use a slave select pin; the AMIS-30543 requires it. */
    void init(uint8_t slaveSelectPin) { ssPin = slaveSelectPin;

        digitalWrite(ssPin, HIGH);
        pinMode(ssPin, OUTPUT);
    }

    /*! Reads the register at the given address and returns its raw value. */
    uint8_t readReg(uint8_t address)
    {
        selectChip();
        transfer(address & 0b11111);
        uint8_t dataOut = transfer(0);
        deselectChip();
        return dataOut;
    }

    /*! Writes the specified value to a register. */
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

/*! This class provides high-level functions for controlling an AMIS-30543
 *  micro-stepping motor driver.
 *
 * It provides access to all the features of the AMIS-30543 SPI interface
 * except the watchdog timer. */
class AMIS30543
{
public:
    /*! The default constructor. */
    AMIS30543()
    {
        wr = cr0 = cr1 = cr2 = cr3 = 0;
    }

    /*! Possible arguments to setStepMode(). */
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

    /*! Bitmasks for the return value of readNonLatchedStatusFlags(). */
    enum nonLatchedStatusFlag
    {
        OPENY = (1 << 2),
        OPENX = (1 << 3),
        WD = (1 << 4),
        CPFAIL = (1 << 5),
        TW = (1 << 6),
    };

    /*! Bitmasks for the return value of readLatchedStatusFlagsAndClear(). */
    enum latchedStatusFlag
    {
        OVCXNB = (1 << 3),
        OVCXNT = (1 << 4),
        OVCXPB = (1 << 5),
        OVCXPT = (1 << 6),
        TSD = (1 << 10),
        OVCYNB = (1 << 11),
        OVCYNT = (1 << 12),
        OVCYPB = (1 << 13),
        OVCYPT = (1 << 14),
    };

    /*! Addresses of control and status registers. */
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

    /*! Configures this object to use the specified pin as a slave select pin.
     * You must use a slave select pin; the AMIS-30543 requires it. */
    void init(uint8_t slaveSelectPin)
    {
        driver.init(slaveSelectPin);
    }

    /*! Changes all of the driver's settings back to their default values.
     *
     * It is good to call this near the beginning of your program to ensure that
     * There are no settings left over from an earlier time that might affect the
     * operation of the driver. */
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
        return driver.readReg(WR) == wr &&
            driver.readReg(CR0) == cr0 &&
            driver.readReg(CR1) == cr1 &&
            driver.readReg(CR2) == cr2 &&
            driver.readReg(CR3) == cr3;
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
        driver.writeReg(CR2, cr2);

        writeWR();
        writeCR0();
        writeCR1();
        writeCR3();
    }

    /*! Sets the MOTEN bit to 1, enabling the driver.
     *
     * Please read the note about this function in README.md. */
    void enableDriver()
    {
        cr2 |= 0b10000000;
        applySettings();
    }

    /*! Sets the MOTEN bit to 0, disabling the driver.
     *
     * Please read the note about this function in README.md. */
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
        uint8_t sr3 = readStatusReg(SR3);
        uint8_t sr4 = readStatusReg(SR4);
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

    /*! Returns the cached value of the DIRCTRL configuration bit.
     *
     * This does not perform any SPI communication with the driver. */
    bool getDirection()
    {
        return cr1 >> 7 & 1;
    }

    /*! Configures the driver to have the specified stepping mode.
     *
     * This affects many things about the performance of the motor, including
     * how much the output moves for each step taken and how much current flows
     * through the coils in each stepping position.
     *
     * The argument to this function should be one of the members of the
     * #stepMode enum.
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

    /*! Reads the status flags from registers SR0.  These flags are not latched,
     * which means they will be cleared as soon as the condition causing them is
     * no longer detected.  See the AMIS-30543 datasheet for more information.
     *
     * This function returns the raw value of SR0, with the parity bit set to 0.
     * You can simply compare the return value to 0 to see if any of the status
     * flags are set, or you can use the logical AND operator (`&`) and the
     * #nonLatchedStatusFlag enum to check individual flags.
     *
     * ~~~~{.cpp}
     * uint16_t flags = stepper.readNonLatchedStatusFlags();
     * if (flags)
     * {
     *   // At least one flag is set.
     *
     *   if (flags & AMIS30543::OPENX)
     *   {
     *     // Thermal shutdown flag is set.
     *   }
     *
     * }
     * ~~~~
     */
    uint16_t readNonLatchedStatusFlags()
    {
        return readStatusReg(SR0);
    }

    /*! Reads the latched status flags from registers SR1 and SR2.  They are
     *  cleared as a side effect.
     *
     * The return value is a 16-bit unsigned integer that has one bit for each
     * status flag.  You can simply compare the return value to 0 to see if any
     * of the status flags are set, or you can use the logical and operator (`&`)
     * and the #latchedStatusFlag enum to check individual flags.
     *
     * WARNING: Calling this function clears the latched error bits in SR1 and
     * SR2, which might allow the motor driver outputs to reactivate.  The
     * AMIS-30543 datasheet says "successive reading the SPI Status Registers 1
     * and 2 in case of a short circuit condition, may lead to damage to the
     * drivers". */
    uint16_t readLatchedStatusFlagsAndClear()
    {
        uint8_t sr1 = readStatusReg(SR1);
        uint8_t sr2 = readStatusReg(SR2);
        return (sr2 << 8) | sr1;
    }

protected:

    uint8_t wr, cr0, cr1, cr2, cr3;

    /*! Reads a status register and returns the lower 7 bits (the parity bit is
     *  set to 0 in the return value). */
    uint8_t readStatusReg(uint8_t address)
    {
        // Mask off the parity bit.
        // (Later we might add code here to check the parity
        // bit and record errors.)
        return driver.readReg(address) & 0x7F;
    }

    /*! Writes the cached value of the WR register to the device. */
    void writeWR()
    {
        driver.writeReg(WR, wr);
    }

    /*! Writes the cached value of the CR0 register to the device. */
    void writeCR0()
    {
        driver.writeReg(CR0, cr0);
    }

    /*! Writes the cached value of the CR1 register to the device. */
    void writeCR1()
    {
        driver.writeReg(CR1, cr1);
    }

    /*! Writes the cached value of the CR3 register to the device. */
    void writeCR3()
    {
        driver.writeReg(CR3, cr3);
    }

public:
    /*! This object handles all the communication with the AMIS-30543.  It is
     * only marked as public for the purpose of testing this library; you should
     * not use it in your code. */
    AMIS30543SPI driver;
};
