#include "main.h"

void asyncChassisTask(void *ignore)
{
  if (nextMove != ASYNC_NONE)
  {
    isChassisMoving = true;
    switch (nextMove)
    {
      case ASYNC_MTT_SIMPLE:
        moveToTargetSimple(mttContainer.targetX, mttContainer.targetY, mttContainer.startX, mttContainer.startY, mttContainer.power, mttContainer.startPower, mttContainer.maxErrorX, mttContainer.decelEarly, mttContainer.decelPower, mttContainer.dropEarly, mttContainer.stopType, mttContainer.mode);
        break;
      case ASYNC_MTT_DIS:
        moveToTargetDisSimple(mttContainer.angle, mttContainer.distance, mttContainer.startX, mttContainer.startY, mttContainer.power, mttContainer.startPower, mttContainer.maxErrorX, mttContainer.decelEarly, mttContainer.decelPower, mttContainer.dropEarly, mttContainer.stopType, mttContainer.mode, mttContainer.isDegrees);
        break;
      case ASYNC_TTT_SWEEP:
        sweepTurnToTarget(sweepContainer.targetX, sweepContainer.targetY, sweepContainer.targetAngle, sweepContainer.targetRadius, sweepContainer.turnDir, sweepContainer.power, sweepContainer.isAccurate, sweepContainer.isDegrees);
        break;
      case ASYNC_TTT_ANGLE:
        turnToAngleNew(turnContainer.targetAngle, turnContainer.turnDir, turnContainer.fullPowerRatio, turnContainer.coastPower, turnContainer.stopPowerDiff, turnContainer.harshStop, turnContainer.isDegrees);
        break;
      case ASYNC_TTT_TARGET:
        turnToTargetNew(turnContainer.targetX, turnContainer.targetY, turnContainer.turnDir, turnContainer.fullPowerRatio, turnContainer.coastPower, turnContainer.stopPowerDiff, turnContainer.angleOffset, turnContainer.harshStop, turnContainer.isDegrees);
        break;
    }
    isChassisMoving = false;
    nextMove = ASYNC_NONE;
  }
}
void waitUntilChassisMoveComplete()
{
  while (isChassisMoving) { delay(40); }
}
void initializeAsyncChassisController()
{
  // Stop task if needed
  unsigned int asyncState = taskGetState(asyncChassisHandle);
  if (asyncChassisHandle != NULL && (asyncState == TASK_RUNNING || asyncState == TASK_SLEEPING || asyncState == TASK_SUSPENDED))
    taskDelete(APSTask);

  asyncChassisHandle = taskCreate(asyncChassisTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT + 1);
}
void stopAsyncChassisController()
{
  // Stop task
  unsigned int asyncState = taskGetState(asyncChassisHandle);
  if (asyncChassisHandle != NULL && (asyncState == TASK_RUNNING || asyncState == TASK_SLEEPING || asyncState == TASK_SUSPENDED))
    taskDelete(APSTask);

  nextMove = ASYNC_NONE;
  isChassisMoving = false;
}
void moveToTargetSimpleAsync(double targetX, double targetY, double startX, double startY, int power, int startPower, double maxErrorX, double decelEarly, int decelPower, double dropEarly, StopType stopType, MTTMode mode)
{
  // Set all motion parameters
  mttContainer.targetX = targetX;
  mttContainer.targetY = targetY;
  mttContainer.startX = startX;
  mttContainer.startY = startY;
  mttContainer.power = power;
  mttContainer.startPower = startPower;
  mttContainer.maxErrorX = maxErrorX;
  mttContainer.decelEarly = decelEarly;
  mttContainer.decelPower = decelPower;
  mttContainer.dropEarly = dropEarly;
  mttContainer.stopType = stopType;
  mttContainer.mode = mode;
  // Set nextMove and run the controller
  nextMove = ASYNC_MTT_SIMPLE;
  initializeAsyncChassisController();
}
void moveToTargetDisSimpleAsync(double angle, double distance, double startX, double startY, int power, int startPower, double maxErrorX, double decelEarly, int decelPower, double dropEarly, StopType stopType, MTTMode mode, bool isDegrees)
{
  // Set all motion parameters
  mttContainer.angle = angle;
  mttContainer.distance = distance;
  mttContainer.startX = startX;
  mttContainer.startY = startY;
  mttContainer.power = power;
  mttContainer.startPower = startPower;
  mttContainer.maxErrorX = maxErrorX;
  mttContainer.decelEarly = decelEarly;
  mttContainer.decelPower = decelPower;
  mttContainer.dropEarly = dropEarly;
  mttContainer.stopType = stopType;
  mttContainer.mode = mode;
  mttContainer.isDegrees = isDegrees;
  // Set nextMove and run the controller
  nextMove = ASYNC_MTT_DIS;
  initializeAsyncChassisController();
}
void sweepTurnToTargetAsync(double targetX, double targetY, double targetAngle, double targetRadius, TurnDir turnDir, int power, bool isAccurate, bool isDegrees)
{
  // Set all motion parameters
  sweepContainer.targetX = targetX;
  sweepContainer.targetY = targetY;
  sweepContainer.targetAngle = targetAngle;
  sweepContainer.targetRadius = targetRadius;
  sweepContainer.turnDir = turnDir;
  sweepContainer.power = power;
  sweepContainer.isAccurate = isAccurate;
  sweepContainer.isDegrees = isDegrees;
  // Set nextMove and run the controller
  nextMove = ASYNC_TTT_SWEEP;
  initializeAsyncChassisController();
}
void turnToAngleNewAsync(double targetAngle, TurnDir turnDir, double fullPowerRatio, int coastPower, double stopPowerDiff, bool harshStop, bool isDegrees)
{
  // Set all motion parameters
  turnContainer.targetAngle = targetAngle;
  turnContainer.turnDir = turnDir;
  turnContainer.fullPowerRatio = fullPowerRatio;
  turnContainer.coastPower = coastPower;
  turnContainer.stopPowerDiff = stopPowerDiff;
  turnContainer.harshStop = harshStop;
  turnContainer.isDegrees = isDegrees;
  // Set nextMove and run the controller
  nextMove = ASYNC_TTT_ANGLE;
  initializeAsyncChassisController();
}
void turnToTargetNewAsync(double targetX, double targetY, TurnDir turnDir, double fullPowerRatio, int coastPower, double stopPowerDiff, double angleOffset, bool harshStop, bool isDegrees)
{
  // Set all motion parameters
  turnContainer.targetX = targetX;
  turnContainer.targetY = targetY;
  turnContainer.turnDir = turnDir;
  turnContainer.fullPowerRatio = fullPowerRatio;
  turnContainer.coastPower = coastPower;
  turnContainer.stopPowerDiff = stopPowerDiff;
  turnContainer.angleOffset = angleOffset;
  turnContainer.harshStop = harshStop;
  turnContainer.isDegrees = isDegrees;
  // Set nextMove and run the controller
  nextMove = ASYNC_TTT_TARGET;
  initializeAsyncChassisController();
}