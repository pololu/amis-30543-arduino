# AMIS-30543 library for Arduino

Version: 1.0.0<br/>
Release date: 2015 May 21<br/>
[www.pololu.com](https://www.pololu.com/)

## Summary

This is a C++ library for the Arduino IDE that helps interface with an [AMIS-30543 micro-stepping stepper motor driver](https://www.pololu.com/product/2970).  It uses the [Arduino SPI](http://www.arduino.cc/en/Reference/SPI) library to communicate with the SPI interface (CS, DO, DI, and CLK) of the AMIS-30543.

## Supported platforms

This library is designed to work with the Arduino IDE versions 1.6.x or later and will probably not work with older versions.  For instance, it does not work with version 1.0.6.  This library supports any Arduino-compatible board, including the [Pololu A-Star 32U4 controllers](https://www.pololu.com/category/149/a-star-programmable-controllers).

## Getting started

### Hardware

An [AMIS-30543 carrier](https://www.pololu.com/product/2970) can be purchased from Pololu's website.  Before continuing, careful reading of the [product page](https://www.pololu.com/product/2970) as well as the AMIS-30543 datasheet is recommended.

You will need to connect your motor, motor power, and IOREF as described on the product page.  You should also make the following connections between the Arduino and the driver:

    Arduino pin 10  - driver STEP
    Arduino pin 9   - driver CS
    Arduino SCK     - driver CLK
    Arduino MOSI    - driver DI
    Arduino MISO    - driver DO   (optional)
    Arduino GND     - driver GND

The DO pin is only needed if you want to read information back from the stepper driver.

The SPI pins (MOSI, MISO, and SCK) on Arduino-compatible boards are sometimes not labeled.  You should refer to the documentation for your particular board to find the locations of these pins.

On the [Arduino Uno](https://www.pololu.com/product/2191), [Arduino Leonardo](https://www.pololu.com/product/2192), and the [A-Star 32U4 controllers](https://www.pololu.com/category/149/a-star-programmable-controllers), the SPI pins (SCK, MOSI, and MISO) can be found on the 6-pin ISP header.

The [Arduino Uno](https://www.pololu.com/product/2191) has additional access points for the SPI pins: pin 11 is MOSI, pin 12 is MISO, and pin 13 is SCK.

### Software

If you are using version 1.6.2 or later of the [Arduino IDE](http://www.arduino.cc/en/Main/Software), you can use the Library Manager to install this library:

1. In the Arduino IDE, open the "Sketch" menu, select "Include Library", then "Manage Libraries...".
2. Search for "AMIS30543".
3. Click the AMIS30543 entry in the list.
4. Click "Install".

If this does not work, you can manually install the library:

1. Download the [latest release archive from GitHub](https://github.com/pololu/amis-30543-arduino/releases) and decompress it.
2. Rename the folder "amis-30543-arduino-xxxx" to "AMIS30543".
3. Drag the "AMIS30543" folder into the "libraries" directory inside your Arduino sketchbook directory.  You can view your sketchbook location by opening the **"File"** menu and selecting **"Preferences"** in the Arduino IDE.  If there is not already a "libraries" folder in that location, you should make the folder yourself.
4. After installing the library, restart the Arduino IDE.

## Examples

Several example sketches are available that show how to use the library. You can
access them from the Arduino IDE by opening the "File" menu, selecting
"Examples", and then selecting "AMIS30543". If you cannot find these
examples, the library was probably installed incorrectly and you should retry
the installation instructions above.

## Documentation

For complete documentation of this library, including many features that were
not mentioned here, see
[the amis-30543-arduino documentation](https://pololu.github.io/amis-30543-arduino/).

## Handling interruptions in stepper motor power

If the power supply to the AMIS-30543 drops too low, then the driver will turn off and the motor will stop moving.  After power is restored, all of the device's configuration registers will have their default values.  This means that the motor outputs will be off and the motor will not turn, even after power is restored.

It is possible to detect this situation using the `verifySettings()` function and/or recover from it using the `applySettings()` function.

The `verifySettings()` function reads all of the configuration registers from the driver and verifies that they match the previously-specified settings.  If `verifySettings()` returns false, then it means that the settings on the driver do not match the cached setting in the `AMIS30543` object, or the device is powered off, or the SPI connections between the microcontroller and the AMIS-30543 are incorrect.  If your SPI connections are correct and the `AMIS30543` object is on the part of your system that might modify the registers on the driver, then the most likely causes for `verifySettings()` returning false would be that the driver is not powered or it lost power at some point in the past.

The `applySettings()` function writes all of the cached settings to the driver.  If the power to the driver is interrupted and then restored, calling `applySettings()` can restore the desired settings to the device and allow it to run again.

The functions `sleep()`, `sleepStop()`, `setSlaGainDefault()`, `setSlaGainHalf()`, `setSlaTransparencyOff()`, `setSlaTransparencyOn()`, `disableDriver()`, and `enableDriver()` each modify bits in CR2, a register that also contains the bit that enables the motor driver outputs (MOTEN).  To help avoid operating the motor at incorrect settings, each of these functions includes a call to `applySettings()`.  When you call any of these functions, your system will automatically recover from earlier interruptions in the stepper motor power.  These functions might also make it hard for you to detect power interruptions by calling `verifySettings()`, because they might automatically fix the settings before `verifySettings()` gets called.

The AutoRecover example that comes with this library shows how to use `verifySettings()` to detect power loss and use `applySettings()` to recover from it.

## Detecting an open coil

**WARNING:** Disconnecting a stepper motor coil while the motor is operating can cause damage to the stepper motor driver or other parts of your system.  If you want to test that open coil detection is working, we recommend removing the connection while the stepper motor driver is not powered.

The AMIS-30543 has the ability to detect when current cannot flow through one of your stepper motor coils.  This is called an open coil, and it usually is caused by incorrect or incomplete connections between the stepper motor and the driver.  It can also be caused by setting a current limit that is too high to be reached with your choice of stepper motor and power supply.

The `readNonLatchedStatusFlags()` function reads the non-latched status flags from the device, including the OPENX and OPENY bits which indicate than an open coil condition has been detected.  The AutoRecover example that comes with this library shows how to use that function.

An open coil condition is only detected after a motor output PWM signal has been at a 100% duty cycle for 200 milliseconds.  Therefore, to detect an open coil, you will need to step the motor sufficiently slowly or you will need to pause the motor's movement for at least 200 ms occasionally.  When you pause, you should be sure to pause at a step position where the desired currents in both coils are non-zero.  Also, after detecting an open coil, you should avoid stepping the motor until the problem is resolved and the status flags change back to 0.  Taking steps during an open coil condition will usually cause the driver to stop detecting the open coil condition and clear the OPENX and OPENY bits.

## Version history

* 1.0.0 (2015 May 21): Original release.
