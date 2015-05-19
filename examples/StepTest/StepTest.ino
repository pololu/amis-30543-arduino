#include <SPI.h>
#include <AMIS30543.h>
#include <AStar32U4Prime.h>  // TODO: remove

const uint8_t amisSlaveSelect = 10;
const uint8_t amisStepPin = 9;

AMIS30543 stepper;

AStar32U4PrimeButtonA buttonA;
AStar32U4PrimeButtonB buttonB;
AStar32U4PrimeButtonC buttonC;

void setup()
{
  ledYellow(0);
  delay(1000);
  Serial.begin(9600);
  Serial.println("start");

  SPI.begin();  // initialize SPI

  pinMode(amisStepPin, OUTPUT);

  Serial.println("AMIS SPI test");
  while (1)
  {
    if (buttonB.getSingleDebouncedRelease())
    {
      Serial.println("button pushed");
      break;
    }
  }
  ledYellow(1);

  stepper.init(amisSlaveSelect);
  stepper.setCurrentMilliamps(800);
  stepper.enableDriver();
}

void loop()
{
  // The NXT minimum high/low pulse width is 2 microseconds.
  digitalWrite(amisStepPin, HIGH);
  delayMicroseconds(3);
  digitalWrite(amisStepPin, LOW);
  delayMicroseconds(3);

  // There is also a limit to how fast you can actually step the
  // stepper motor.
  delay(1);
}
