/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

void deploy()
{
  moveArmsMedAsync();
  waitUntilArmMoveComplete();
  moveArmsZeroAsync();
  delay(1000);
}
void score()
{
  // Variables to make sure robot is against wall
  unsigned long stopTimer = millis();
  bool isStopped = false;

  // Keep driving forwards until robot is still for 350 milliseconds
  while (!isStopped)
  {
    setDriveLinear(40, 40);

    if (!isRobotStopped())
      stopTimer = millis();

    if (millis() - stopTimer > 350)
    {
      isStopped = true;
      setDrive(5, 5);
    }
  }

  // Tilt the stack vertical for scoring
  moveTrayVerticalAsync();
  delay(2000);
  setRollers(-40);
}
void autoSkillsSuperSafe()
{
  // Reset to proper pose on the field and deploy
  resetPositionFull(&globalPose, 0.0, 0.0, 0.0, true);
	resetVelocity(&globalVel, globalPose);

  // 1.
  moveToTargetSimpleAsync(0.0, -8.0, 0.0, 0.0, -127, 0, 2.0, 0, 0, 0, STOP_SOFT, MTT_SIMPLE);

  // 2.
  moveToTargetSimple(0.0, 0.0, 0.0, -8.0, 127, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
}
void autoBlueSmallSafe()
{
  // 0. Reset to proper pose on the field and deploy
  resetPositionFull(&globalPose, 26.4, BACK_TO_CENTER, 0.0, true);
	resetVelocity(&globalVel, globalPose);
  deploy();

  // 1. Start rollers and drive forward to collect preload and 4 cubes
  setRollers(127);
  moveToTargetSimpleAsync(26.4, 50.0, 26.4, BACK_TO_CENTER, 60, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
  waitUntilChassisMoveComplete();

  // 2. Stop rollers and drive backwards to diagonal for scoring
  setRollers(60);
  stopAsyncArmController();
  moveToTargetSimpleAsync(26.4, 26.4, 26.4, globalPose.y, -80, 0, 0.5, 0, 20, 0, STOP_SOFT, MTT_CASCADING);
  waitUntilChassisMoveComplete();

  // 3. Turn towards small goal
  turnToTargetNewAsync(0.0, 0.0, TURN_CCW, 0.6, 25, 10, 0.0, true, true);
  waitUntilChassisMoveComplete();

  // 4. Drive towards goal, and score the stack
  moveToTargetSimpleAsync(15.0, 15.0, 26.4, 26.4, 100, 0, 0.5, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
  waitUntilChassisMoveComplete();
  score();

  // 5. Back away from the stack and tilt the tray back and return the arms down
  moveToTargetDisSimpleAsync(45.0, 18.0, 12.0, 12.0, -50, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL, true);
  delay(1000);
  moveTrayAngledAsync();
  moveArmsZeroAsync();
  waitUntilChassisMoveComplete();
}
void autoBlueSmallSuperSafe()
{
  // 0. Reset to proper pose on the field
  resetPositionFull(&globalPose, 0, 0, 0.0, true);
	resetVelocity(&globalVel, globalPose);

  // 1.
  moveToTargetSimpleAsync(0.0, -8.0, 0.0, 0.0, -127, 0, 2.0, 0, 0, 0, STOP_SOFT, MTT_SIMPLE);

  // 2.
  moveToTargetSimple(-24.0, 16.0, 0.0, -8.0, 127, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
  deploy();
}
void autoBlueLargeSuperSafe()
{
  // 0. Reset to proper pose on the field
  resetPositionFull(&globalPose, 0, 0, 0.0, true);
	resetVelocity(&globalVel, globalPose);

  // 1.
  moveToTargetSimpleAsync(0.0, -8.0, 0.0, 0.0, -127, 0, 2.0, 0, 0, 0, STOP_SOFT, MTT_SIMPLE);

  // 2.
  moveToTargetSimple(24.0, 16.0, 0.0, -8.0, 127, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
  deploy();
}
void autoRedSmallSafe()
{
  // 0. Reset to proper pose on the field and deploy
  resetPositionFull(&globalPose, FIELD_WIDTH - 26.4, BACK_TO_CENTER, 0.0, true);
	resetVelocity(&globalVel, globalPose);
  deploy();

  // 1. Start rollers and drive forward to collect preload and 4 cubes
  setRollers(127);
  moveToTargetSimpleAsync(FIELD_WIDTH - 26.4, 50.0, FIELD_WIDTH - 26.4, BACK_TO_CENTER, 60, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
  waitUntilChassisMoveComplete();

  // 2. Stop rollers and drive backwards to diagonal for scoring
  setRollers(0);
  moveToTargetSimpleAsync(FIELD_WIDTH - 26.4, 26.4, FIELD_WIDTH - 26.4, globalPose.y, -127, 0, 0.5, 0, 20, 0, STOP_SOFT, MTT_CASCADING);
  waitUntilChassisMoveComplete();

  // 3. Turn towards small goal
  turnToTargetNewAsync(FIELD_WIDTH, 0.0, TURN_CW, 0.6, 25, 10, 0.0, true, true);
  waitUntilChassisMoveComplete();

  // 4. Drive towards goal, and score the stack
  moveToTargetSimpleAsync(FIELD_WIDTH - 15.0, 15.0, FIELD_WIDTH - 26.4, 26.4, 100, 0, 0.5, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
  waitUntilChassisMoveComplete();
  score();

  // 5. Back away from the stack and tilt the tray back and return the arms down
  moveToTargetDisSimpleAsync(-45.0, 12.0, FIELD_WIDTH - 12.0, 12.0, -30, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL, true);
  delay(1000);
  moveTrayAngled();
  moveArmsZero();
  waitUntilChassisMoveComplete();
}
void autoRedSmallSuperSafe()
{
  // 0. Reset to proper pose on the field
  resetPositionFull(&globalPose, 0, 0, 0.0, true);
	resetVelocity(&globalVel, globalPose);

  // 1.
  moveToTargetSimpleAsync(0.0, -8.0, 0.0, 0.0, -127, 0, 2.0, 0, 0, 0, STOP_SOFT, MTT_SIMPLE);

  // 2.
  moveToTargetSimple(24.0, 16.0, 0.0, -8.0, 127, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
  deploy();
}
void autoRedLargeSuperSafe()
{
  // 0. Reset to proper pose on the field
  resetPositionFull(&globalPose, 0, 0, 0.0, true);
	resetVelocity(&globalVel, globalPose);

  // 1.
  moveToTargetSimpleAsync(0.0, -8.0, 0.0, 0.0, -127, 0, 2.0, 0, 0, 0, STOP_SOFT, MTT_SIMPLE);

  // 2.
  moveToTargetSimple(-24.0, 16.0, 0.0, -8.0, 127, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
  deploy();
}
void autonomous()
{
  // Choose the correct autonomous
  switch (chosenAuto)
  {
    case AUTO_BLUE_SMALL:
      autoBlueSmallSuperSafe();
      break;
    case AUTO_BLUE_LARGE:
      autoBlueLargeSuperSafe();
      break;
    case AUTO_RED_SMALL:
      autoRedSmallSuperSafe();
      break;
    case AUTO_RED_LARGE:
      autoRedLargeSuperSafe();
      break;
    case AUTO_SKILLS:
      autoSkillsSuperSafe();
      break;
    case AUTO_NONE:
      break;
  }
}

void autoChooser()
{
  // Turn on all LEDS
  digitalWrite(PORT_redAllianceLED, LOW);
  digitalWrite(PORT_smallGoalLED, LOW);
  digitalWrite(PORT_skillsLED, LOW);

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
      digitalWrite(PORT_skillsLED, HIGH);
      break;
    case AUTO_BLUE_LARGE:
      digitalWrite(PORT_redAllianceLED, HIGH);
      digitalWrite(PORT_smallGoalLED, HIGH);
      digitalWrite(PORT_skillsLED, HIGH);
      break;
    case AUTO_RED_SMALL:
      digitalWrite(PORT_redAllianceLED, LOW);
      digitalWrite(PORT_smallGoalLED, LOW);
      digitalWrite(PORT_skillsLED, HIGH);
      break;
    case AUTO_RED_LARGE:
      digitalWrite(PORT_redAllianceLED, LOW);
      digitalWrite(PORT_smallGoalLED, HIGH);
      digitalWrite(PORT_skillsLED, HIGH);
      break;
    case AUTO_SKILLS:
      digitalWrite(PORT_redAllianceLED, HIGH);
      digitalWrite(PORT_smallGoalLED, HIGH);
      digitalWrite(PORT_skillsLED, LOW);
      break;
    case AUTO_NONE:
      digitalWrite(PORT_redAllianceLED, LOW);
      digitalWrite(PORT_smallGoalLED, LOW);
      digitalWrite(PORT_skillsLED, LOW);
      break;
  }
  delay(500);
}
