## Handling loss of stepper motor power

If the power supply to the AMIS-30543 drops too low, then the driver will turn off and the motor will stop moving.  After power is restored, all of the device's configuration registers will have their default values.  This means that the motor outputs will be off and the motor will not turn, even after power is restored.

It is possible to detect this situation using the `verifySettings()` function and/or recover from it using the `applySettings()` function.

The `verifySettings()` function reads all of the configuration registers from the driver and verifies that they match the previously-specified settings.  If `verifySettings()` returns false, then it means that the settings on the driver do not match the cached setting in the `AMIS30543` object, or the device is powered off, or the SPI connections between the microcontroller and the AMIS-30543 are incorrect.  If your SPI connections are correct and the `AMIS30543` object is on the part of your system that might modify the registers on the driver, then the most likely causes for `verifySettings()` returning false would be that the driver is not powered or it lost power at some point in the past.

The `applySettings()` function writes the cached settings to the driver.  If the power to the driver is interrupted and then restored, calling `applySettings()` can restore the desired settings to the device and allow it to run again.

The AutoRecover example that comes with this library shows how to use `verifySettings()` to detect power loss and use `applySettings()` to recover from it.
