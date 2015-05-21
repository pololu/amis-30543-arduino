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

const uint8_t amisSlaveSelect = 10;
const uint8_t amisStepPin = 9;

AMIS30543 stepper;

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
  // The NXT minimum high pulse width is 2 microseconds.
  digitalWrite(amisStepPin, HIGH);
  delayMicroseconds(3);
  digitalWrite(amisStepPin, LOW);
  delayMicroseconds(3);

  // The delay here controls the stepper motor's speed.  You can
  // increase the delay to make the stepper motor go slower.
  delayMicroseconds(2000);
}
