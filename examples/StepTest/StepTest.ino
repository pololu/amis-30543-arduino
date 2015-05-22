/* This code is used to test all of the features of the AMIS30543
library.  It is mainly intended to be used by developers of the
library.

Power supply: 10 V, 1 A

Scope setup:
- Channel 1: pin 1
- Channel 2: SLA
- Channel 3: MOTXP
- Trigger on a falling edge of channel 1.

*/

const uint8_t scopeTriggerPin = 1;
const uint8_t amisClrPin = 3;
const uint8_t amisSlaveSelect = 4;
const uint8_t amisStepPin = 5;

const bool skipAutoTests = false;

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

  digitalWrite(scopeTriggerPin, LOW);
  pinMode(scopeTriggerPin, OUTPUT);

  Serial.begin(9600);
  SPI.begin();
  pinMode(amisStepPin, OUTPUT);
  stepper.init(amisSlaveSelect);

  if (!skipAutoTests)
  {
    Serial.println(F("Press enter to start the test."));
    waitForSerial();

    // Automated tests.
    testResetSettings();
    testVerifySettings();
    testReadStatusFlags();
    testEnableDisableDriver();
    testCurrentLimit();
    testSteppingAndReadingPosition();
    testDirControl();
    testNXTP();
    testStepModes();
    testSleep();
    Serial.println(F("All automated tests passed."));
  }

  digitalWrite(13, HIGH); // Turn on the yellow LED.

  testWithScope();
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
    Serial.println(F("Writing or reading WR failed; driver power might be off."));
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

void testVerifySettings()
{
  resetDriver();

  if (!stepper.verifySettings())
  {
    Serial.println(F("Failed to verify empty settings."));
    error();
  }

  writeReg(WR, 0x20);

  if (stepper.verifySettings())
  {
    Serial.println(F("verifySettings failed to detect that a register changed."));
    error();
  }
}

void testReadStatusFlags()
{
  resetDriver();

  uint8_t nonLatchedFlags = stepper.readNonLatchedStatusFlags();
  uint16_t latchedFlags = stepper.readLatchedStatusFlagsAndClear();
  if (nonLatchedFlags || latchedFlags)
  {
    Serial.println(F("A status flag was set."));
    error();
  }

  // Set WDEN = 1 to enable the watchdog timer (32 ms).
  writeReg(WR, 0x80);

  delay(50);

  nonLatchedFlags = stepper.readNonLatchedStatusFlags();
  if (nonLatchedFlags != AMIS30543::WD)
  {
    Serial.println(F("Unexpected status flags."));
    Serial.println(nonLatchedFlags);
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

void testCurrentLimit()
{
  resetDriver();

  stepper.setCurrentMilliamps(800);
  if (readCUR() != 0b01010)
  {
    Serial.println(F("setCurrentLimit(800) failed"));
  }

  stepper.setCurrentMilliamps(244);
  if (readCUR() != 0b00000)
  {
    Serial.println(F("setCurrentLimit(244) failed"));
  }
  stepper.setCurrentMilliamps(3000);
  if (readCUR() != 0b11001)
  {
    Serial.println(F("setCurrentLimit(3000) failed"));
  }
}

void testSteppingAndReadingPosition()
{
  resetDriver();
  stepper.enableDriver();

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
  uint16_t pos[12];

  // Note: This test is not good enough to discern any differences between
  // the 3 different full-step modes or the 2 different half-step modes.

  resetDriver();
  stepper.resetSettings();
  stepper.enableDriver();

  stepper.setStepMode(AMIS30543::MicroStep32);
  nextStep(); pos[0] = stepper.readPosition();

  stepper.setStepMode(16);
  nextStep(); pos[1] = stepper.readPosition();

  stepper.setStepMode(8);
  nextStep(); pos[2] = stepper.readPosition();

  stepper.setStepMode(4);
  nextStep(); pos[3] = stepper.readPosition();

  stepper.setStepMode(AMIS30543::CompensatedHalf);
  nextStep(); pos[4] = stepper.readPosition();

  stepper.setStepMode(AMIS30543::CompensatedFullTwoPhaseOn);
  nextStep(); pos[5] = stepper.readPosition();
  nextStep(); pos[6] = stepper.readPosition();
  // This is kind of a contradiction; it's in "two-phase on"
  // mode right now but one of the coils will be pretty much off.

  stepper.setStepMode(AMIS30543::UncompensatedHalf);
  nextStep(); pos[7] = stepper.readPosition();

  stepper.setStepMode(AMIS30543::CompensatedFullOnePhaseOn);
  nextStep(); pos[8] = stepper.readPosition();

  stepper.setStepMode(128);
  nextStep(); pos[9] = stepper.readPosition();

  stepper.setStepMode(AMIS30543::UncompensatedFull);
  nextStep(); pos[10] = stepper.readPosition();

  stepper.setStepMode(64);
  nextStep(); pos[11] = stepper.readPosition();

  if (pos[0] != 4 || pos[1] != 12 || pos[2] != 28
    || pos[3] != 60 || pos[4] != 124
    || pos[5] != 252 || pos[6] != 380
    || pos[7] != 444 || pos[8] != 60
    || pos[9] != 61 || pos[10] != 189
    || pos[11] != 191)
  {
    Serial.println(F("SM: Microstep positions are wrong."));
    for(uint8_t i = 0; i < sizeof(pos)/sizeof(pos[0]); i++)
    {
      Serial.println(pos[i]);
    }
    error();
  }
}

void testSleep()
{
  resetDriver();
  stepper.enableDriver();

  stepper.sleep();
  uint8_t cr2 = readReg(CR2);
  if (cr2 != 0xC0)
  {
    Serial.println("sleepStart failed: bad CR2 value.");
    Serial.println(cr2, HEX);
    stepper.resetSettings();  // avoid getting stuck in sleep mode
    error();
  }

  delay(1);
  // Driving CLR high does nothing because of sleep mode,
  // so MOTEN will stay set.
  digitalWrite(amisClrPin, HIGH);
  delay(1);
  digitalWrite(amisClrPin, LOW);
  delay(1);

  stepper.sleepStop();
  cr2 = readReg(CR2);
  if (cr2 != 0x80)
  {
    Serial.println("sleepStop failed: bad CR2 value.");
    Serial.println(cr2, HEX);
    stepper.resetSettings();  // avoid getting stuck in sleep mode
    error();
  }
}

void testWithScope()
{
  Serial.println(F("Check oscilloscope."));

  resetDriver();
  stepper.setStepMode(4);
  stepper.setCurrentMilliamps(500);
  stepper.enableDriver();

  // NOTE: We can't really test setPwmJitterOn and
  // setPwmJitterOff here.  We could have a second mode where you
  // would have to set the scope to trigger on the motor channel,
  // leave it at a constant current, and turn the jitter feature
  // on and off every second or something.  Here we only make
  // sure they can compile and they don't break anything.
  stepper.setPwmJitterOn();
  stepper.setPwmJitterOff();

  uint8_t count = 0;

  while(1)
  {
    static uint8_t lastMode = 0xFF;
    uint8_t mode = count >> 6 & 3;

    if (lastMode != mode)
    {
      lastMode = mode;

      // By triggering on channel 3 and zooming in to 100 ns/div,
      // verify that the PWM fall (or rise) time increases every 2
      // seconds, for a total of 4 different settings.
      stepper.setPwmSlope(mode);

      // Verify that the SLA gain changes every two seconds.
      // (The two settings are 0.5 and 0.25.)
      if (mode & 1)
      {
        stepper.setSlaGainHalf();
      }
      else
      {
        stepper.setSlaGainDefault();
      }

      // Verify that the SLA transparency setting changes every 4 seconds.
      if (mode & 2)
      {
        stepper.setSlaTransparencyOn();
      }
      else
      {
        stepper.setSlaTransparencyOff();
      }

      // Verify that the system can detect and recover nicely from an
      // interruption to stepper motor power.  (It will have a lag of
      // 0-2 seconds because we only do this check every 2 seconds.)
      if (!stepper.verifySettings())
      {
        stepper.applySettings();
        if (!stepper.verifySettings())
        {
          Serial.println(F("Could not verify settings; driver power might be off."));
        }
      }
    }

    cli();

    digitalWrite(scopeTriggerPin, HIGH);
    delayMicroseconds(100);
    digitalWrite(scopeTriggerPin, LOW);
    delayMicroseconds(10);

    // First step:
    // Verify that the PWM frequency seen on the motor output
    // is doubled (50 kHz).
    stepper.setPwmFrequencyDouble();
    nextStep();
    delay(1);

    // Second step:
    // Verify that the PWM frequency seen on the motor output
    // is normal (25 kHz).
    stepper.setPwmFrequencyDefault();
    nextStep();
    delay(1);

    sei();

    // Get back to the starting position.
    for (uint8_t i = 0; i < 30; i++)
    {
      nextStep();
      delay(1);
    }

    count++;
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

uint8_t readCUR()
{
  return readReg(CR0) & 0b11111;
}

void resetDriver()
{
  digitalWrite(amisClrPin, HIGH);
  delay(1);
  digitalWrite(amisClrPin, LOW);
  delay(1);
  stepper.resetSettings();
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

void error()
{
  stepper.disableDriver();
  while(1)
  {
  }
}
