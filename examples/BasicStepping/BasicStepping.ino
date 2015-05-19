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
  stepper.setCurrentMilliamps(500);
  stepper.enableDriver();
}

void loop()
{
  // The NXT minimum high pulse width is 2 microseconds.
  digitalWrite(amisStepPin, HIGH);
  delayMicroseconds(3);
  digitalWrite(amisStepPin, LOW);
  delayMicroseconds(3);

  delay(1);
}
