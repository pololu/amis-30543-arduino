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
