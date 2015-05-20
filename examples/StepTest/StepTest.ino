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

// When developing theses tests for the AMIS-30543, we found that
// a delay of 15 microseconds between changing NXT and reading
// the microstepping position was too short (resulting in getting
// the old microstepping position instead of the new one).  A
// delay of 25 microseconds seems to be sufficient.  This isn't
// documented in the datasheet; the closest thing is tNXT_HI,
// which is 2 microseconds, and a claim that "The translator
// position is updated immediately following a NXT trigger."
const uint16_t postStepDelayUs = 100;

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
  testEnableDisableDriver();
  testSteppingAndReadingPosition();
  testDirControl();
  testNXTP();
  testStepModes();

  success();
}

void loop()
{
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

void testEnableDisableDriver()
{
  resetDriver();
  stepper.enableDriver();
  if (stepper.driver.readReg(CR2) != 0x80)
  {
    Serial.println(F("Error: EnableDriver failed."));
    error();
  }
  stepper.disableDriver();
  if (stepper.driver.readReg(CR2) != 0x00)
  {
    Serial.println(F("Error: DisableDriver failed."));
    error();
  }
}

void testSteppingAndReadingPosition()
{
  resetDriver();
  stepper.enableDriver();

  Serial.println(F("Test readPosition"));

  uint16_t pos0 = stepper.readPosition();
  nextStep();
  uint16_t pos1 = stepper.readPosition();
  nextStep();
  uint16_t pos2 = stepper.readPosition();

  if (pos0 != 0 || pos1 != 4 || pos2 != 8)
  {
    Serial.println(F("ReadPosition: Microstep positions are wrong."));
    Serial.println(pos0);
    Serial.println(pos1);
    Serial.println(pos2);
    error();
  }
}

void testDirControl()
{
  resetDriver();
  stepper.enableDriver();
  stepper.setDirection(1);
  nextStep();
  uint16_t pos0 = stepper.readPosition();
  stepper.setDirection(0);
  nextStep();
  uint16_t pos1 = stepper.readPosition();

  if (pos0 != 508 || pos1 != 0)
  {
    Serial.println(F("DIRCTRL: Microstep positions are wrong."));
    Serial.println(pos0);
    Serial.println(pos1);
    error();
  }
}

void testNXTP()
{
  resetDriver();
  stepper.enableDriver();

  digitalWrite(amisStepPin, HIGH);
  delayMicroseconds(postStepDelayUs);
  uint16_t pos0 = stepper.readPosition();

  stepper.stepOnFallingEdge();
  digitalWrite(amisStepPin, LOW);
  delayMicroseconds(postStepDelayUs);
  uint16_t pos1 = stepper.readPosition();

  stepper.stepOnRisingEdge();
  digitalWrite(amisStepPin, HIGH);
  delayMicroseconds(postStepDelayUs);
  uint16_t pos2 = stepper.readPosition();

  digitalWrite(amisStepPin, LOW);
  delayMicroseconds(postStepDelayUs);
  uint16_t pos3 = stepper.readPosition();

  if (pos0 != 4 || pos1 != 8 || pos2 != 12 || pos3 != 12)
  {
    Serial.println(F("NXTP: Microstep positions are wrong."));
    Serial.println(pos0);
    Serial.println(pos1);
    Serial.println(pos2);
    Serial.println(pos3);
    error();
  }
}

void testStepModes()
{
  resetDriver();
  stepper.resetSettings();
  stepper.enableDriver();

  stepper.setStepMode(AMIS30543::MicroStep32);
  nextStep();
  uint16_t pos0 = stepper.readPosition();

  stepper.setStepMode(16);
  nextStep();
  uint16_t pos1 = stepper.readPosition();

  // TODO: finish testing all the different modes

  if (pos0 != 4 || pos1 != 12)
  {
    Serial.println(F("SM: Microstep positions are wrong."));
    Serial.println(pos0);
    Serial.println(pos1);
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

uint8_t readStatusReg(uint8_t address)
{
  return readReg(address) & 0x7F;
}

void resetDriver()
{
  digitalWrite(amisClrPin, HIGH);
  delay(1);
  digitalWrite(amisClrPin, LOW);
  delay(1);
}

void nextStep()
{
  // The NXT minimum high pulse width is 2 microseconds.
  digitalWrite(amisStepPin, HIGH);
  delayMicroseconds(3);
  digitalWrite(amisStepPin, LOW);
  delayMicroseconds(3);

  delayMicroseconds(postStepDelayUs);
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
