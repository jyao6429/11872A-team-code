/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

#define MAX_INTAKE_CHASSIS_V 45

AutoOptions chosenAuto;
uint32_t autoTimer;

// Functions
void deploy()
{
  moveArmsMedAsync();
  pros::delay(200);
  setRollers(-127);
  waitUntilArmMoveComplete(2000);
  pros::delay(300);
  moveArmsZeroAsync();
  pros::delay(500);
  setRollers(127);
  waitUntilArmMoveComplete(2000);
}
void autoScoreSmall(AutoColor alliance, bool needsOuttake, bool needsReset)
{
  // Variables for alliance specific motions
  double XCoord = 0.0;
  TurnDir turn = TURN_CCW;
  if (alliance == AUTO_COLOR_RED)
  {
    XCoord = FIELD_WIDTH - XCoord;
    turn = TURN_CW;
  }

  // 1. Turn towards small goal
  turnToTargetNewAsync(XCoord, 0.0, turn, 0.7, 30, 10, 0.0, true, true);
  waitUntilChassisMoveComplete(4000, 250, false);

  // 2. Drive towards small goal, and score the stack
  XCoord = 12.0;
  double XCoord1 = 26.4;
  if (alliance == AUTO_COLOR_RED)
  {
    XCoord = FIELD_WIDTH - XCoord;
    XCoord1 = FIELD_WIDTH - XCoord1;
  }

  // 5. Outtake if needed
  if (needsOuttake)
  {
    outtakeOneCubeAsync();
  }

  moveToTargetSimpleAsync(XCoord, 12.0, XCoord1, 26.4, 110, 0, 0.5, 0, 0, 0, STOP_NONE, MTT_CASCADING);
  waitUntilChassisMoveComplete(2000, 200, true);

  // Variables to make sure robot is against wall
  unsigned long stopTimer = pros::millis();
  bool isStopped = true;

  // 4. Keep driving forwards until robot is still for 250 milliseconds
  while (!isStopped)
  {
    setDrive(60, 60);

    if (!isRobotStopped())
      stopTimer = pros::millis();

    if (pros::millis() - stopTimer > 250)
    {
      isStopped = true;
      stopDrive();
    }
    pros::delay(10);
  }

  // 6. Tilt the stack vertical and outtake to drop stack for scoring
  moveTrayVerticalAsync();
  while (getTrayPot() < 1400)
  {
    // Handle outtake when stacking
		if (getTrayPot() > 1000 && getTrayPot() < 1400 && nextTrayTarget == TRAY_VERTICAL)
		{
			setRollers(-55);
		}
    pros::delay(10);
  }
  stopRollers();
  waitUntilTrayMoveComplete(2000);

  // 6.5. Push forward a bit
  setDrive(30, 30);
  pros::delay(500);

  // 7. Back up and move tray back
  setDrive(-70, -70);
  moveTrayAngledAsync();
  pros::delay(600);
  stopDrive();
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
  moveToTargetSimpleAsync(XCoord, 13.0, XCoord, BACK_TO_CENTER, 127, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_SIMPLE);
  waitUntilChassisMoveComplete(2000, 250, true);
  moveToTargetSimpleAsync(XCoord, 49.0, XCoord, BACK_TO_CENTER, MAX_INTAKE_CHASSIS_V, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_SIMPLE);
  waitUntilChassisMoveComplete(6000, 250, true);

  if (getFifth)
  {
    double XCoord1 = 35.0;
    if (alliance == AUTO_COLOR_RED)
      XCoord1 = FIELD_WIDTH - XCoord1;
    moveToTargetSimpleAsync(XCoord1, 55.0, XCoord, 26.4, MAX_INTAKE_CHASSIS_V, MAX_INTAKE_CHASSIS_V, 1, 0, 0, 0, STOP_SOFT, MTT_SIMPLE);
    waitUntilChassisMoveComplete(2000, 250, true);
  }

  if (backUp)
  {
    // 2. Drive backwards to diagonal for scoring
    moveToTargetSimpleAsync(XCoord, 26.4, XCoord, globalPose.y, -127, 0, 1.5, 0, 20, 0, STOP_HARSH, MTT_CASCADING);
    waitUntilChassisMoveComplete(4000, 250, true);
  }
}
void autoRunOfThree(AutoColor alliance, bool backUpTo4, bool get2Stack)
{
  // Variable used for alliance
  double XCoord = 50.4;
  if (alliance == AUTO_COLOR_RED)
    XCoord = FIELD_WIDTH - XCoord;

  // 1. Start rollers and drive forward to collect preload and 4 cubes
  moveArmsZeroAsync();
  setRollers(127);
  moveToTargetSimpleAsync(XCoord, 18.0, XCoord, BACK_TO_CENTER, 127, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_SIMPLE);
  waitUntilChassisMoveComplete(400, 250, false);
  moveToTargetSimpleAsync(XCoord, 37.0, XCoord, BACK_TO_CENTER, MAX_INTAKE_CHASSIS_V, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_SIMPLE);
  waitUntilChassisMoveComplete(4000, 250, true);

  if (get2Stack)
  {
    // 2. Move arms up to grab second cube and bring down to lower cube
    moveArmsSecondAsync();
    pros::delay(300);
    moveToTargetSimpleAsync(XCoord, 54.0, XCoord, BACK_TO_CENTER, MAX_INTAKE_CHASSIS_V, MAX_INTAKE_CHASSIS_V, 1.0, 0, 0, 0, STOP_NONE, MTT_SIMPLE);
    waitUntilArmMoveComplete(1000);
    moveArmsZeroAsync();
    waitUntilArmMoveComplete(500);
    waitUntilChassisMoveComplete(3000, 250, true);
  }

  if (backUpTo4)
  {
    // 3. Back up a bit
    moveToTargetSimpleAsync(XCoord, 44.0, XCoord, 54.0, -127, 0, 1.0, 0, 0, 0, STOP_NONE, MTT_SIMPLE);
    waitUntilChassisMoveComplete(2000, 250, false);

    // 4. Drive diagonally to run of 4
    double XCoord1 = 26.4;
    TurnDir turn1 = TURN_CW;
    TurnDir turn2 = TURN_CCW;
    if (alliance == AUTO_COLOR_RED)
    {
      XCoord1 = FIELD_WIDTH - XCoord1;
      turn1 = TURN_CCW;
      turn2 = TURN_CW;
    }
    turnToTargetNew(XCoord1, 10.0, turn1, 0.7, 30, 10, 180.0, true, true);
    waitUntilChassisMoveComplete(2000, 250, false);
    moveToTargetSimpleAsync(XCoord1, 10.0, XCoord, 44.0, -127, 0, 2.0, 0, 0, 0, STOP_SOFT, MTT_SIMPLE);
    waitUntilChassisMoveComplete(5000, 250, false);

    // 5. Turn to face forwards
    turnToAngleNewAsync(0.0, turn2, 0.7, 30, 10, true, false);
    waitUntilChassisMoveComplete(2000, 250, true);
  }
}


void test5Pt()
{
  resetPositionFull(&globalPose, FIELD_WIDTH - 50.4, BACK_TO_CENTER, 0.0, true);
	resetVelocity(&globalVel, globalPose);
  deploy();
  autoRunOfThree(AUTO_COLOR_RED, true, false);
  autoRunOfFour(AUTO_COLOR_RED, true, false);
  autoScoreSmall(AUTO_COLOR_RED, true, false);
}
void testStack()
{
  resetPositionFull(&globalPose, 26.4, 26.4, -135.0, true);
	resetVelocity(&globalVel, globalPose);
  autoScoreSmall(AUTO_COLOR_BLUE, true, false);
}
void testKill()
{
  resetPositionFull(&globalPose, 0.0, 0.0, 0.0, true);
	resetVelocity(&globalVel, globalPose);
  moveToTargetSimpleAsync(100, 0, 0.0, 0.0, 30, 0, 2.0, 0, 0, 0, STOP_SOFT, MTT_SIMPLE);
  //waitUntilChassisMoveComplete(400, 600, true);
}
// Testing scripts
void autoTest()
{
  //test5Pt();
  testKill();
  //testStack();
  //outtakeOneCubeAsync();
  //waitUntilRollerMoveComplete(1000);
}


// Skills scripts
void autoSkills()
{
  // Reset to proper pose on the field and deploy
  resetPositionFull(&globalPose, 50.4, BACK_TO_CENTER, 0.0, true);
	resetVelocity(&globalVel, globalPose);
  deploy();

  // 1. 9 Pt stack and reset
  autoRunOfThree(AUTO_COLOR_BLUE, true, true);
  autoRunOfFour(AUTO_COLOR_BLUE, true, false);
  autoScoreSmall(AUTO_COLOR_BLUE, true, true);
}
// Small goal scripts
void autoSmall9Pt(AutoColor alliance)
{
  double XCoord = 50.4;
  if (alliance == AUTO_COLOR_RED)
    XCoord = FIELD_WIDTH - XCoord;

  // Reset to proper pose on the field and deploy
  resetPositionFull(&globalPose, XCoord, BACK_TO_CENTER, 0.0, true);
	resetVelocity(&globalVel, globalPose);
  deploy();
  autoRunOfThree(alliance, true, true);
  autoRunOfFour(alliance, true, false);
  autoScoreSmall(alliance, true, false);
}
void autoSmall8Pt(AutoColor alliance)
{
  double XCoord = 50.4;
  if (alliance == AUTO_COLOR_RED)
    XCoord = FIELD_WIDTH - XCoord;

  // Reset to proper pose on the field and deploy
  resetPositionFull(&globalPose, XCoord, BACK_TO_CENTER, 0.0, true);
	resetVelocity(&globalVel, globalPose);
  deploy();
  autoRunOfThree(alliance, true, false);
  autoRunOfFour(alliance, true, true);
  autoScoreSmall(alliance, true, false);
}
void autoSmall7Pt(AutoColor alliance)
{
  double XCoord = 50.4;
  if (alliance == AUTO_COLOR_RED)
    XCoord = FIELD_WIDTH - XCoord;

  // Reset to proper pose on the field and deploy
  resetPositionFull(&globalPose, XCoord, BACK_TO_CENTER, 0.0, true);
	resetVelocity(&globalVel, globalPose);
  deploy();
  autoRunOfThree(alliance, true, false);
  autoRunOfFour(alliance, true, false);
  //autoScoreSmall(alliance, true, false);
}
void autoSmall6Pt(AutoColor alliance)
{
  double XCoord = 26.4;
  if (alliance == AUTO_COLOR_RED)
    XCoord = FIELD_WIDTH - XCoord;

  // Reset to proper pose on the field and deploy
  resetPositionFull(&globalPose, XCoord, BACK_TO_CENTER, 0.0, true);
	resetVelocity(&globalVel, globalPose);
  deploy();
  autoRunOfFour(alliance, true, true);
  autoScoreSmall(alliance, true, false);
}
void autoSmall5Pt(AutoColor alliance)
{
  double XCoord = 26.4;
  if (alliance == AUTO_COLOR_RED)
    XCoord = FIELD_WIDTH - XCoord;

  // Reset to proper pose on the field and deploy
  resetPositionFull(&globalPose, XCoord, BACK_TO_CENTER, 0.0, true);
	resetVelocity(&globalVel, globalPose);
  deploy();
  autoRunOfFour(alliance, true, false);
  autoScoreSmall(alliance, true, false);
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
  moveToTargetSimple(0.0, 0.0, 0.0, 0.0, 127, 0, 2.0, 0, 0, 0, STOP_SOFT, MTT_SIMPLE);

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
