/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

void autonomous()
{
}

void autoChooser()
{
  unsigned long timer = millis();
  bool notChosen = true;

  // Gives 5 seconds to choose or end early by hitting button
  while (millis() - timer < 5000 && notChosen)
  {
    int currentRead = analogRead(PORT_dialPot);

    // Choose auton based on 5 sections of the pot
    if (currentRead < 819)
      chosenAuto = AUTO_BLUE_SMALL;
    else if (currentRead < 1638)
      chosenAuto = AUTO_BLUE_LARGE;
    else if (currentRead < 2457)
      chosenAuto = AUTO_SKILLS;
    else if (currentRead < 3276)
      chosenAuto = AUTO_RED_LARGE;
    else
      chosenAuto = AUTO_RED_SMALL;

    // Disengage if interact button is held for half a second
    if (digitalRead(PORT_interactButton) == LOW)
    {
      delay(500);
      notChosen = digitalRead(PORT_interactButton) == HIGH;
    }

    delay(20);
  }
}
