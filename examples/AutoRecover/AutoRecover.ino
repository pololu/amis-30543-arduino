/* This example shows how to detect and automatically recover
from various errors that can occur when using the AMIS-30543,
including an interruption of the motor power supply.

Before using this example, be sure to change the parameter to
setCurrentMilliamps to an appropriate current limit for your
system.  Also, see this library's documentation for information
about how to connect the driver:

    http://pololu.github.io/amis-30543-arduino/
*/

#include <SPI.h>
#include <AMIS30543.h>

const uint8_t amisSlaveSelect = 9;
const uint8_t amisStepPin = 10;

AMIS30543 stepper;

bool stepperSettingsProblem = false;
uint8_t stepperNonLatchedFlags = 0;
uint16_t stepperLatchedFlags = 0;

void setup()
{
  SPI.begin();
  pinMode(amisStepPin, OUTPUT);
  delay(1);

  stepper.init(amisSlaveSelect);
  stepper.resetSettings();
  stepper.setCurrentMilliamps(245);
  stepper.enableDriver();
}

void loop()
{
  takeSteps();
  checkDriver();
}

void takeSteps()
{
  // Don't take any steps if there is an issue.
  if (stepperSettingsProblem || stepperNonLatchedFlags || stepperLatchedFlags)
  {
    return;
  }

  if ((millis() & 1023) >= 774)
  {
    // Pause for 250 ms once per second.
  }
  else
  {
    // Otherwise, take a step roughly every 1500 microseconds.
    static uint16_t lastStepTime = 0;
    if ((uint16_t)(micros() - lastStepTime) >= 1500)
    {
      lastStepTime = micros();

      // The NXT minimum high pulse width is 2 microseconds.
      digitalWrite(amisStepPin, HIGH);
      delayMicroseconds(3);
      digitalWrite(amisStepPin, LOW);
      delayMicroseconds(3);
    }
  }
}

void checkDriver()
{
  // Every 20 ms, perform some checks.
  static uint16_t lastCheckTime = 0;
  if ((uint16_t)(millis() - lastCheckTime) >= 20)
  {
    // Read back the configuration of the driver and make sure it
    // is correct.
    if (!stepperSettingsProblem && !stepper.verifySettings())
    {
      // We have detected the settings on the driver do not match
      // the desired settings we chose earlier in this sketch.
      // It is likely that stepper motor power has been lost or
      // that communication with the driver is not working.
      stepperSettingsProblem = true;
      Serial.println(F("Could not verify settings; driver power might be off."));
    }
    if (stepperSettingsProblem)
    {
      // The settings are currently not correct.  Try writing the
      // correct settings to the device to fix it.  This will not
      // work if the device is powered off.
      stepper.applySettings();

      if (stepper.verifySettings())
      {
        // We successfully restored the settings.
        stepperSettingsProblem = false;
        Serial.println(F("Successfully reapplied settings."));
      }
    }

    // Read the non-latched status flags.  This should allow us
    // to detect thermal warnings, charge pump failure, watchdog
    // events, and open coil conditions.
    uint8_t newFlags = stepper.readNonLatchedStatusFlags();
    if (newFlags != stepperNonLatchedFlags)
    {
      stepperNonLatchedFlags = newFlags;
      Serial.print(F("Non-latched status flags changed: 0x"));
      Serial.print(newFlags, HEX);
      Serial.println();
    }

    lastCheckTime = millis();
  }

  // Every 500 ms, perform some other checks.
  static uint16_t lastSlowCheckTime = 0;
  if ((uint16_t)(millis() - lastSlowCheckTime) >= 500)
  {
    lastSlowCheckTime = millis();

    // Read the latched status flags.  Note that this has the
    // effect of clearing the latched flags and allowing the
    // motor driver outputs to operate again.  If there is a
    // short circuit, reading these flags too frequently could
    // cause damage to the driver.  That is why we only read it
    // twice per second in this example.  If you want to get
    // faster notifications of latched errors, you could monitor
    // the ERR pin.
    uint16_t newFlags = stepper.readLatchedStatusFlagsAndClear();
    if (newFlags != stepperLatchedFlags)
    {
      stepperLatchedFlags = newFlags;
      Serial.print(F("Latched status flags changed: 0x"));
      Serial.print(newFlags, HEX);
      Serial.println();
    }
  }

}
