#include "main.h"

// Handles the async task
std::unique_ptr<pros::Task> asyncChassisHandle;
// Variables for handling async tasks
// Keeps track when the robot is performing a move
bool isChassisMoving;
// The next move to be performed by the robot
AsyncChassisOptions nextChassisMove;
// Container variables for each motion
MTTContainer mttContainer;
TTTSweepContainer sweepContainer;
TTTRegularContainer turnContainer;

void asyncChassisTask(void *ignore)
{
  // Store the current move
  AsyncChassisOptions currentChassisMove = ASYNC_CHASSIS_NONE;
  mutexes[MUTEX_ASYNC_CHASSIS].take(1000);
  currentChassisMove = nextChassisMove;
  mutexes[MUTEX_ASYNC_CHASSIS].give();

  // Switch between each motion type
  switch (currentChassisMove)
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
    case ASYNC_CHASSIS_NONE:
      break;
  }
  // Reset variables
  mutexes[MUTEX_ASYNC_CHASSIS].take(200);
  nextChassisMove = ASYNC_CHASSIS_NONE;
  isChassisMoving = false;
  mutexes[MUTEX_ASYNC_CHASSIS].give();
}
bool waitUntilChassisMoveComplete(int timeout, int stoppedTime, bool needsStop)
{
  uint32_t timer = pros::millis();
  uint32_t stopTimer = pros::millis();

  bool isOverride = false;
  while (isChassisMoving)
  {
    if (pros::millis() - timer > timeout)
    {
      isOverride = true;
      break;
    }

    if (!isRobotStopped())
      stopTimer = pros::millis();

    if (pros::millis() - stopTimer > stoppedTime)
    {
      isOverride = true;
      break;
    }

    pros::delay(40);
  }
  if (isOverride && needsStop)
    stopAsyncChassisController();

  return isOverride;
}
bool waitUntilChassisMoveComplete(int timeout, bool needsStop)
{
  return waitUntilChassisMoveComplete(timeout, 1000000, needsStop);
}
void waitUntilChassisMoveComplete()
{
  while (isChassisMoving) { pros::delay(40); }
}
void queueAsyncChassisController(AsyncChassisOptions moveToQueue)
{
  // Stop task if needed
  if (asyncChassisHandle != NULL && (asyncChassisHandle->get_state() != pros::E_TASK_STATE_DELETED) && (asyncChassisHandle->get_state() != pros::E_TASK_STATE_INVALID))
    asyncChassisHandle->remove();
  stopDrive();

  // Checks if there is an actual motion to do
  if (moveToQueue == ASYNC_CHASSIS_NONE)
    return;

  // Start the task
  mutexes[MUTEX_ASYNC_CHASSIS].take(2000);
  nextChassisMove = moveToQueue;
  isChassisMoving = true;
  asyncChassisHandle = std::make_unique<pros::Task>(asyncChassisTask, nullptr, TASK_PRIORITY_DEFAULT + 1);
  mutexes[MUTEX_ASYNC_CHASSIS].give();
}
void stopAsyncChassisController()
{
  // Stop task if needed
  if (asyncChassisHandle != NULL && (asyncChassisHandle->get_state() != pros::E_TASK_STATE_DELETED) && (asyncChassisHandle->get_state() != pros::E_TASK_STATE_INVALID))
    asyncChassisHandle->remove();
  stopDrive();

  // Reset variables
  mutexes[MUTEX_ASYNC_CHASSIS].take(500);
  nextChassisMove = ASYNC_CHASSIS_NONE;
  isChassisMoving = false;
  mutexes[MUTEX_ASYNC_CHASSIS].give();
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
  // Queue the next move
  queueAsyncChassisController(ASYNC_MTT_SIMPLE);
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
  // Queue the next move
  queueAsyncChassisController(ASYNC_MTT_DIS);
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
  // Queue the next move
  queueAsyncChassisController(ASYNC_TTT_SWEEP);
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
  // Queue the next move
  queueAsyncChassisController(ASYNC_TTT_ANGLE);
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
  // Queue the next move
  queueAsyncChassisController(ASYNC_TTT_TARGET);
}
