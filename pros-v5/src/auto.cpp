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

void deploy()
{

  moveArmsZeroAsync();
  moveTrayVerticalAsync();
  //delay(1500);
  //setRollers(-127);
  waitUntilTrayMoveComplete();
  //delay(750);
  //moveTrayAngledAsync();
  //waitUntilTrayMoveComplete();
  //stopRollers();
}
void score()
{
  // Variables to make sure robot is against wall
  unsigned long stopTimer = pros::millis();
  bool isStopped = false;

  // Keep driving forwards until robot is still for 350 milliseconds
  while (!isStopped)
  {
    setDrive(0.4, 0.4);

    if (!isRobotStopped())
      stopTimer = pros::millis();

    if (pros::millis() - stopTimer > 350)
    {
      isStopped = true;
      setDrive(0.05, 0.05);
    }
  }

  // Tilt the stack vertical for scoring
  moveTrayVerticalAsync();
  waitUntilTrayMoveComplete();
}
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
