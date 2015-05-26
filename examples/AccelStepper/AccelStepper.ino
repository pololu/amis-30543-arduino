/* This example shows how to use the AMIS-30543 stepper motor
driver with the AccelStepper library.

You will need to install the AccelStepper library for this
example to work.  Documentation and installation instructions for
the AccelStepper library are available here:

    http://www.airspayce.com/mikem/arduino/AccelStepper/

Before using this example, be sure to change the
setCurrentMilliamps line to have an appropriate current limit for
your system.  Also, see this library's documentation for
information about how to connect the driver:

    http://pololu.github.io/amis-30543-arduino/
*/

#include <SPI.h>
#include <AMIS30543.h>
#include <AccelStepper.h>

const uint8_t amisDirPin = 2;
const uint8_t amisStepPin = 3;
const uint8_t amisSlaveSelect = 4;

AMIS30543 stepper;
AccelStepper accelStepper(AccelStepper::DRIVER, amisStepPin, amisDirPin);

void setup()
{
  SPI.begin();
  stepper.init(amisSlaveSelect);
  delay(1);

  stepper.resetSettings();
  stepper.setCurrentMilliamps(132);
  stepper.setStepMode(32);
  stepper.enableDriver();

  accelStepper.setMaxSpeed(2000.0);
  accelStepper.setAcceleration(500.0);
}

void loop()
{
  accelStepper.runToNewPosition(0);
  delay(500);
  accelStepper.runToNewPosition(10000);
  delay(500);
}
