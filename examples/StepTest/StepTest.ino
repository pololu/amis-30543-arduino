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
  delay(1000);
  Serial.begin(9600);
  Serial.println("start");

  SPI.begin();  //initialize SPI
  stepper.init(amisSlaveSelect);

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
