/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

#define MAX_INTAKE_CHASSIS_V 50

AutoOptions chosenAuto;

// Functions
void deploy()
{
  moveArmsLowAsync();
  waitUntilArmMoveComplete(1000);
  moveArmsZeroAsync();
  setRollers(127);
  waitUntilArmMoveComplete();
}
void autoScoreSmall(AutoColor alliance, bool needsOuttake)
{
  // Variables for alliance specific motions
  double XCoord = 0.0;
  if (alliance == AUTO_COLOR_RED)
    XCoord = FIELD_WIDTH - XCoord;

  // 1. Turn towards small goal
  turnToTargetNewAsync(XCoord, 0.0, TURN_CH, 0.6, 25, 10, 0.0, true, true);
  waitUntilChassisMoveComplete();

  // 2. Drive towards small goal, and score the stack
  XCoord = 14.0;
  double XCoord1 = 26.4;
  if (alliance == AUTO_COLOR_RED)
  {
    XCoord = FIELD_WIDTH - XCoord;
    XCoord1 = FIELD_WIDTH - XCoord1;
  }
  moveToTargetSimpleAsync(XCoord, 14.0, XCoord1, 26.4, 100, 0, 0.5, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
  waitUntilChassisMoveComplete();

  // Variables to make sure robot is against wall
  unsigned long stopTimer = pros::millis();
  bool isStopped = false;

  // 4. Keep driving forwards until robot is still for 250 milliseconds
  while (!isStopped)
  {
    setDrive(60, 60);

    if (!isRobotStopped())
      stopTimer = pros::millis();

    if (pros::millis() - stopTimer > 250)
    {
      isStopped = true;
      setDrive(5, 5);
    }
    pros::delay(10);
  }

  // 5. Outtake if needed
  if (needsOuttake)
  {
    setRollers(-50);
    pros::delay(600);
    stopRollers();
  }

  // 6. Tilt the stack vertical for scoring
  moveTrayVerticalAsync();
  waitUntilTrayMoveComplete();

  // 7. Back up and move tray back while outtaking
  setDrive(-60, -60);
  setRollersVel(-100);
  moveTrayAngledAsync();
  pros::delay(1000);
  stopDrive();
  stopRollers();
}
void autoRunOfFour(AutoColor alliance, bool backUp, bool getFifth)
{
  // Variable used for alliance
  double XCoord = 26.4;
  if (alliance == AUTO_COLOR_RED)
    XCoord = FIELD_WIDTH - XCoord;

  // 1. Start rollers and drive forward to collect preload and 4 cubes
  moveArmsZeroAsync();
  setRollers(127);
  moveToTargetSimpleAsync(XCoord, 54.0, XCoord, BACK_TO_CENTER, MAX_INTAKE_CHASSIS_V, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_SIMPLE);
  waitUntilChassisMoveComplete();

  if (getFifth)
  {

  }

  if (backUp)
  {
    // 2. Drive backwards to diagonal for scoring
    moveToTargetSimpleAsync(XCoord, 26.4, XCoord, globalPose.y, -100, 0, 0.5, 0, 20, 0, STOP_SOFT, MTT_CASCADING);
    waitUntilChassisMoveComplete();
  }
}
void autoRunOfThree(AutoColor alliance, bool backUp)
{
  // Variable used for alliance
  double XCoord = 26.4;
  if (alliance == AUTO_COLOR_RED)
    XCoord = FIELD_WIDTH - XCoord;

  // 1. Start rollers and drive forward to collect preload and 4 cubes
  moveArmsZeroAsync();
  setRollers(127);
  moveToTargetSimpleAsync(XCoord, 54.0, XCoord, BACK_TO_CENTER, MAX_INTAKE_CHASSIS_V, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_SIMPLE);
  waitUntilChassisMoveComplete();
}

// Small goal scripts
void autoSmall9Pt(AutoColor alliance)
{
  double XCoord = 0;
  // Reset to proper pose on the field and deploy
  resetPositionFull(&globalPose, 0.0, 0.0, 0.0, true);
	resetVelocity(&globalVel, globalPose);
  deploy();
  autoRunOfThree(alliance, true);
  autoRunOfFour(alliance, true, false);
  autoScoreSmall(alliance, true);
}
// Large goal scripts
// Shared scripts
void autoOnePt()
{
  // Reset to proper pose on the field
  resetPositionFull(&globalPose, 0.0, 0.0, 0.0, true);
	resetVelocity(&globalVel, globalPose);

  // 1. Back up and forward
  moveToTargetSimple(0.0, -8.0, 0.0, 0.0, -127, 0, 2.0, 0, 0, 0, STOP_SOFT, MTT_SIMPLE);
  moveToTargetSimple(0.0, 8.0, 0.0, 0.0, 127, 0, 2.0, 0, 0, 0, STOP_SOFT, MTT_SIMPLE);

  // 2. Deploy
  deploy();
}

// OLD STUFF
void autoSkillsSuperSafe()
{
  // Reset to proper pose on the field and deploy
  resetPositionFull(&globalPose, 0.0, 0.0, 0.0, true);
	resetVelocity(&globalVel, globalPose);

  // 1.
  moveToTargetSimpleAsync(0.0, -8.0, 0.0, 0.0, -127, 0, 2.0, 0, 0, 0, STOP_SOFT, MTT_SIMPLE);
  waitUntilChassisMoveComplete();

  // 2.
  moveToTargetSimple(0.0, 0.0, 0.0, -8.0, 127, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
}
void autoBlueSmallSafe()
{
  // 0. Reset to proper pose on the field, then push preload forward and deploy
  resetPositionFull(&globalPose, 26.4, BACK_TO_CENTER, 0.0, true);
	resetVelocity(&globalVel, globalPose);
  deploy();

  // 1. Start rollers and drive forward to collect preload and 4 cubes
  setRollers(1.0);
  moveToTargetSimpleAsync(26.4, 54.0, 26.4, BACK_TO_CENTER, MAX_INTAKE_CHASSIS_V, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_SIMPLE);
  waitUntilChassisMoveComplete();
  //moveToTargetSimpleAsync(35.0, 55.0, 50.0, 26.4, MAX_INTAKE_CHASSIS_V, MAX_INTAKE_CHASSIS_V, 1, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
  //waitUntilChassisMoveComplete();

  // 2. Slow rollers and drive backwards to diagonal for scoring
  moveToTargetSimpleAsync(26.4, 26.4, 26.4, globalPose.y, -100, 0, 0.5, 0, 20, 0, STOP_SOFT, MTT_CASCADING);
  waitUntilChassisMoveComplete();

  // 3. Turn towards small goal
  turnToTargetNewAsync(0.0, 0.0, TURN_CCW, 0.6, 25, 10, 0.0, true, true);
  waitUntilChassisMoveComplete();

  // 4. Drive towards small goal, and score the stack
  moveToTargetSimpleAsync(14.0, 14.0, 26.4, 26.4, 100, 0, 0.5, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
  waitUntilChassisMoveComplete();
  setRollers(-1.0);
  pros::delay(625);
  setRollers(1.0);
  pros::delay(200);
  stopRollers();
  moveTrayVerticalAsync();
  waitUntilTrayMoveComplete();

  setDrive(0.5, 0.5);
  pros::delay(500);


  //score();

  // 5. Back away from the stack and tilt the tray back
  moveToTargetDisSimpleAsync(45.0, 12.0, 12.0, 12.0, -40, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL, true);
  waitUntilChassisMoveComplete();
  moveTrayAngledAsync();

}
void autoBlueSmallSuperSafe()
{
  // 0. Reset to proper pose on the field
  resetPositionFull(&globalPose, 0, 0, 0.0, true);
	resetVelocity(&globalVel, globalPose);

  // 1.
  moveToTargetSimpleAsync(0.0, -8.0, 0.0, 0.0, -127, 0, 2.0, 0, 0, 0, STOP_SOFT, MTT_SIMPLE);
  waitUntilChassisMoveComplete();

  // 2.
  moveToTargetSimple(-12.0, 12.0, 0.0, -8.0, 127, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
  deploy();
}
void autoBlueLargeSuperSafe()
{
  // 0. Reset to proper pose on the field
  resetPositionFull(&globalPose, 0, 0, 0.0, true);
	resetVelocity(&globalVel, globalPose);

  // 1.
  moveToTargetSimpleAsync(0.0, -8.0, 0.0, 0.0, -127, 0, 2.0, 0, 0, 0, STOP_SOFT, MTT_SIMPLE);
  waitUntilChassisMoveComplete();

  // 2.
  moveToTargetSimple(12.0, 12.0, 0.0, -8.0, 127, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
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
  moveToTargetSimpleAsync(FIELD_WIDTH - 26.4, 54.0, FIELD_WIDTH - 26.4, BACK_TO_CENTER, MAX_INTAKE_CHASSIS_V, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_SIMPLE);
  waitUntilChassisMoveComplete();
  //moveToTargetSimpleAsync(35.0, 55.0, 50.0, 26.4, MAX_INTAKE_CHASSIS_V, MAX_INTAKE_CHASSIS_V, 1, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
  //waitUntilChassisMoveComplete();

  // 2. Slow rollers and drive backwards to diagonal for scoring
  moveToTargetSimpleAsync(FIELD_WIDTH - 26.4, 26.4, FIELD_WIDTH - 26.4, globalPose.y, -100, 0, 0.5, 0, 20, 0, STOP_SOFT, MTT_CASCADING);
  waitUntilChassisMoveComplete();

  // 3. Turn towards small goal
  turnToTargetNewAsync(FIELD_WIDTH, 0.0, TURN_CW, 0.6, 25, 10, 0.0, true, true);
  waitUntilChassisMoveComplete();

  // 4. Drive towards small goal, and score the stack
  moveToTargetSimpleAsync(FIELD_WIDTH - 14.0, 14.0, FIELD_WIDTH - 26.4, 26.4, 100, 0, 0.5, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
  waitUntilChassisMoveComplete();
  setRollers(-127);
  pros::delay(625);
  setRollers(127);
  pros::delay(200);
  stopRollers();
  moveTrayVerticalAsync();
  waitUntilTrayMoveComplete();

  setDrive(60, 60);
  pros::delay(500);

  // 5. Back away from the stack and tilt the tray back
  moveToTargetDisSimpleAsync(-45.0, 12.0, FIELD_WIDTH - 12.0, 12.0, -40, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL, true);
  waitUntilChassisMoveComplete();
  moveTrayAngledAsync();
}
void autoRedSmallSuperSafe()
{
  // 0. Reset to proper pose on the field
  resetPositionFull(&globalPose, 0, 0, 0.0, true);
	resetVelocity(&globalVel, globalPose);

  // 1.
  moveToTargetSimpleAsync(0.0, -8.0, 0.0, 0.0, -127, 0, 2.0, 0, 0, 0, STOP_SOFT, MTT_SIMPLE);
  waitUntilChassisMoveComplete();

  // 2.
  moveToTargetSimple(12.0, 12.0, 0.0, -8.0, 127, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
  deploy();
}
void autoRedLargeSuperSafe()
{
  // 0. Reset to proper pose on the field
  resetPositionFull(&globalPose, 0, 0, 0.0, true);
	resetVelocity(&globalVel, globalPose);

  // 1.
  moveToTargetSimpleAsync(0.0, -8.0, 0.0, 0.0, -127, 0, 2.0, 0, 0, 0, STOP_SOFT, MTT_SIMPLE);
  waitUntilChassisMoveComplete();

  // 2.
  moveToTargetSimple(-12.0, 12.0, 0.0, -8.0, 127, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
  deploy();
}
