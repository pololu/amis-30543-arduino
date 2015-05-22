/* This example shows basic use of the AMIS-30543 stepper motor
driver.

It shows how to initialize the dirver, set the current limit, set
the micro-stepping mode, and enable the driver.  It shows how to
send pulses to the step pin to get the driver to take steps and
how to switch directions over SPI.  The DO and DIR pins are not
used and do not need to be connected.

Before using this example, be sure to change the
setCurrentMilliamps line to have an appropriate current limit for
your system.  Also, see this library's documentation for
information about how to connect the driver:

    http://pololu.github.io/amis-30543-arduino/
*/

#include <SPI.h>
#include <AMIS30543.h>

const uint8_t amisSlaveSelect = 9;
const uint8_t amisStepPin = 10;

AMIS30543 stepper;

void setup()
{
  SPI.begin();
  pinMode(amisStepPin, OUTPUT);
  delay(1);

  stepper.init(amisSlaveSelect);
  stepper.resetSettings();
  stepper.setCurrentMilliamps(245);
  stepper.setStepMode(32);
  stepper.enableDriver();
}

void loop()
{
  // Step in the default direction 1000 times.
  stepper.setDirection(0);
  for (unsigned int x = 0; x < 1000; x++)
  {
    step();
  }

  // Wait for 300 ms.
  delay(300);

  // Step in the other direction 1000 times.
  stepper.setDirection(1);
  for (unsigned int x = 0; x < 1000; x++)
  {
    step();
  }

  // Wait for 300 ms.
  delay(300);
}

void step()
{
  // Send a pulse on the NXT/STEP pin to tell the driver to take one step.
  digitalWrite(amisStepPin, HIGH);
  delayMicroseconds(3);
  digitalWrite(amisStepPin, LOW);
  delayMicroseconds(3);

  // The delay here controls the stepper motor's speed.  You can
  // increase the delay to make the stepper motor go slower.
  delayMicroseconds(2000);
}
