/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

void autoSkills()
{

}
void autoBlueSmall()
{

}
void autoBlueLarge()
{

}
void autoRedSmall()
{

}
void autoRedLarge()
{

}
void autonomous()
{
  // Choose the correct autonomous
  switch (chosenAuto)
  {
    case AUTO_BLUE_SMALL:
      autoBlueSmall();
      break;
    case AUTO_BLUE_LARGE:
      autoBlueLarge();
      break;
    case AUTO_RED_SMALL:
      autoRedSmall();
      break;
    case AUTO_RED_LARGE:
      autoRedLarge();
      break;
    case AUTO_SKILLS:
      autoSkills();
      break;
    case AUTO_NONE:
      break;
  }
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

  // Light the correct LED indicators
  switch (chosenAuto)
  {
    case AUTO_BLUE_SMALL:
      digitalWrite(PORT_redAllianceLED, HIGH);
      digitalWrite(PORT_smallGoalLED, LOW);
      break;
    case AUTO_BLUE_LARGE:
      digitalWrite(PORT_redAllianceLED, HIGH);
      digitalWrite(PORT_smallGoalLED, HIGH);
      break;
    case AUTO_RED_SMALL:
      digitalWrite(PORT_redAllianceLED, LOW);
      digitalWrite(PORT_smallGoalLED, LOW);
      break;
    case AUTO_RED_LARGE:
      digitalWrite(PORT_redAllianceLED, LOW);
      digitalWrite(PORT_smallGoalLED, HIGH);
      break;
    case AUTO_SKILLS:
      // Flash both LEDs for 1 sec
      for (int i = 0; i < 5; i++)
      {
        digitalWrite(PORT_redAllianceLED, LOW);
        digitalWrite(PORT_smallGoalLED, LOW);
        delay(100);
        digitalWrite(PORT_redAllianceLED, HIGH);
        digitalWrite(PORT_smallGoalLED, HIGH);
        delay(100);
      }
      break;
    case AUTO_NONE:
      // Flash 1 LED for 1 sec
      for (int i = 0; i < 5; i++)
      {
        digitalWrite(PORT_redAllianceLED, LOW);
        delay(100);
        digitalWrite(PORT_redAllianceLED, HIGH);
        delay(100);
      }
      break;
  }
}
