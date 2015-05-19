/* This code is used to test all of the features of the library. */

#include <SPI.h>
#include <AMIS30543.h>

enum regAddr
{
  WR  = 0x0,
  CR0 = 0x1,
  CR1 = 0x2,
  CR2 = 0x3,
  CR3 = 0x9,
  SR0 = 0x4,
  SR1 = 0x5,
  SR2 = 0x6,
  SR3 = 0x7,
  SR4 = 0xA,
};

const uint8_t amisStepPin = 9;
const uint8_t amisSlaveSelect = 10;
const uint8_t amisClrPin = 11;

AMIS30543 stepper;

void setup()
{
  // Turn off the yellow LED.
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  Serial.begin(9600);
  SPI.begin();
  pinMode(amisStepPin, OUTPUT);
  delay(1);

  Serial.println(F("Press enter to start the test."));
  waitForSerial();

  stepper.init(amisSlaveSelect);

  testResetSettings();
  testEnableDriver();

  success();
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

void testResetSettings()
{
  resetDriver();
  writeReg(WR,  0b01111000);
  writeReg(CR0, 0b00000001);
  writeReg(CR1, 0b10000000);
  writeReg(CR2, 0b00010000);
  writeReg(CR3, 0b00000001);

  if (readReg(WR) != 0b01111000)
  {
    Serial.println(F("Writing or reading WR failed."));
    error();
  }

  if (readReg(CR0) != 0b00000001)
  {
    Serial.println(F("Writing or reading CR0 failed."));
    error();
  }

  if (readReg(CR1) != 0b10000000)
  {
    Serial.println(F("Writing or reading CR1 failed."));
    error();
  }

  if (readReg(CR2) != 0b00010000)
  {
    Serial.println(F("Writing or reading CR2 failed."));
    error();
  }

  if (readReg(CR3) != 0b00000001)
  {
    Serial.println(F("Writing or reading CR3 failed."));
    error();
  }

  stepper.resetSettings();

  if (readReg(WR) || readReg(CR0) || readReg(CR1) || readReg(CR2) || readReg(CR3))
  {
    Serial.println(F("ResetSettings failed."));
    error();
  }
}

void testEnableDriver()
{
  resetDriver();
  stepper.enableDriver();
  if (stepper.driver.readReg(CR2) != 0x80)
  {
    Serial.println(F("Error: EnableDriver failed."));
    error();
  }
}

void writeReg(uint8_t address, uint8_t value)
{
  stepper.driver.writeReg(address, value);
}

uint8_t readReg(uint8_t address)
{
  return stepper.driver.readReg(address);
}

void resetDriver()
{
  digitalWrite(amisStepPin, HIGH);
  delay(1);
  digitalWrite(amisStepPin, LOW);
  delay(1);
}

// Wait for the user to send a newline character on the serial monitor.
void waitForSerial()
{
    while(true)
    {
        if (Serial.available() && Serial.read() == '\n')
        {
            break;
        }
    }
}

void success()
{
    Serial.println("All tests passed.");
    digitalWrite(13, HIGH);
    while(1)
    {
    }
}

void error()
{
    while(1)
    {
    }
}
