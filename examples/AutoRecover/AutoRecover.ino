
      if (!stepper.verifySettings())
      {
        stepper.applySettings();

        if (stepper.verifySettings())
        {
          Serial.println(F("Successfully reapplied settings."));
        }
        else
        {
          Serial.println(F("Could not verify settings; driver power might be off."));
        }
      }
