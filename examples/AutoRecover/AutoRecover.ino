#include <SPI.h>
#include <AMIS30543.h>

const uint8_t amisSlaveSelect = 10;
const uint8_t amisStepPin = 9;

AMIS30543 stepper;

bool stepperSettingsProblem = false;
uint16_t stepperFlags = 0;

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

  checkStepper();
}

void checkStepper()
{
  // Only run this check every 500 ms.
  static uint16_t lastCheckTime = 0;
  if ((millis() - lastCheckTime < 500)) { return; }

  if (!stepperSettingsProblem && !stepper.verifySettings())
  {
    stepperSettingsProblem = true;
    Serial.println(F("Could not verify settings; driver power might be off."));
  }

  if (stepperSettingsProblem)
  {
    stepper.applySettings();

    if (stepper.verifySettings())
    {
      stepperSettingsProblem = false;
      Serial.println(F("Successfully reapplied settings."));
    }
  }

  uint16_t newFlags = stepper.readStatusFlags();
  if (newFlags != stepperFlags)
  {
    // TODO: figure out why this is not detecting open coils; maybe it doesn't work at all
    stepperFlags = newFlags;
    Serial.print(F("Status flags changed: 0x"));
    Serial.print(newFlags, HEX);
    Serial.println();
  }

  lastCheckTime = millis();
}
