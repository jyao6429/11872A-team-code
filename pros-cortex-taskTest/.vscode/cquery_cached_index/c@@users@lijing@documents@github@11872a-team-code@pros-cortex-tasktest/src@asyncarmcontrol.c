#include "main.h"

PID armPID;
bool trayOverride = false;
// Handles the async task
TaskHandle asyncArmHandle;
int prevArmTarget = -1;

void asyncArmLoop()
{
  // Precaution in case mutex is taken
  int currentArmTarget = prevArmTarget;
  int currentArmPot = getArmPot();

  // Take mutexes and set proper variables, depending on if operator control
  if (!isAutonomous() && isEnabled() && isMainConnected)
  {
    if (joystickGetDigital(1, 6, JOY_UP))
		{
			// Press buttons to set arm position
			if (joystickGetDigital(1, 6, JOY_DOWN))
				currentArmTarget = ARM_ZERO;
			else if (joystickGetDigital(1, 5, JOY_DOWN))
				currentArmTarget = ARM_LOW;
			else if (joystickGetDigital(1, 5, JOY_UP))
				currentArmTarget = ARM_MED;
		}
    // If need to kill arm
    else if (joystickGetDigital(1, 8, JOY_UP))
    {
      currentArmTarget = -1;
      prevArmTarget = currentArmTarget;
      mutexTake(mutexes[MUTEX_ASYNC_ARM], 200);
      isArmAtTarget = true;
      mutexGive(mutexes[MUTEX_ASYNC_ARM]);
    }
  }
  else
  {
    mutexTake(mutexes[MUTEX_ASYNC_ARM], 200);
    currentArmTarget = nextArmTarget;
    mutexGive(mutexes[MUTEX_ASYNC_ARM]);
  }

  mutexTake(mutexes[MUTEX_ASYNC_ARM], 200);
  trayOverride = currentArmTarget > 0 && (currentArmTarget != ARM_ZERO || (currentArmTarget == ARM_ZERO && abs(currentArmPot - currentArmTarget) > 1500));
  mutexGive(mutexes[MUTEX_ASYNC_ARM]);

  //print("Arm always printing\n");

  // Disengage if no target set
  if (currentArmTarget < 0)
  {
    stopArms();
    return;
  }

  // Reinitialize PID if needed
  if (currentArmTarget != prevArmTarget)
    pidInit(&armPID, 0.005, 0.001, 0.0);

  // Calculate and set power for arm
  int power = pidCalculate(&armPID, currentArmTarget, currentArmPot) * 127;
  setArms(power);

  // Debug
  printf("armPot: %d\tpower: %d\ttarget: %d\n", currentArmPot, power, currentArmTarget);

  // Set prev variables
  prevArmTarget = currentArmTarget;

  // Sets if arm is within target
  if (abs(currentArmTarget - currentArmPot) < 50)
  {
    mutexTake(mutexes[MUTEX_ASYNC_ARM], 200);
    isArmAtTarget = true;
    mutexGive(mutexes[MUTEX_ASYNC_ARM]);
  }
  else
  {
    mutexTake(mutexes[MUTEX_ASYNC_ARM], 200);
    isArmAtTarget = false;
    mutexGive(mutexes[MUTEX_ASYNC_ARM]);
  }
}
void waitUntilArmMoveComplete()
{
  while (!isArmAtTarget) { delay(40); }
}
void startAsyncArmController()
{
  startAsyncTrayController();

  print("~~~~~~~~~~~~~~Begin startAsyncArmController~~~~~~~~~~~~~~~~\n");
  // Only if task is not already running
  unsigned int asyncState = taskGetState(asyncArmHandle);
  if (asyncArmHandle != NULL && (asyncState != TASK_DEAD))
    return;

  // Take the mutex
  mutexTake(mutexes[MUTEX_ASYNC_ARM], 500);
  // Reset variables
  nextArmTarget = -1;
  isArmAtTarget = true;

  // Initialize the PID
  pidInit(&armPID, 0.005, 0.001, 0.0);
  // Stop the arm
  stopArms();

  // Create the task
  asyncArmHandle = taskRunLoop(asyncArmLoop, 20);
  mutexGive(mutexes[MUTEX_ASYNC_ARM]);

  print("~~~~~~~~~~~~~~Finished startAsyncArmController~~~~~~~~~~~~~~~~\n");

}
void stopAsyncArmController()
{
  // Stop task if needed
  unsigned int asyncState = taskGetState(asyncArmHandle);
  if (asyncArmHandle != NULL && (asyncState != TASK_DEAD))
    taskDelete(asyncArmHandle);

  // Reset variables
  mutexTake(mutexes[MUTEX_ASYNC_ARM], 500);
  nextArmTarget = -1;
  isArmAtTarget = true;
  mutexGive(mutexes[MUTEX_ASYNC_ARM]);

  // Stop the arms
  stopArms();
}
void moveArmsZeroAsync()
{
  // Start asyncArmController if needed
  startAsyncArmController();
  // Set the proper target
  mutexTake(mutexes[MUTEX_ASYNC_ARM], 1000);
  nextArmTarget = ARM_ZERO;
  isArmAtTarget = false;
  mutexGive(mutexes[MUTEX_ASYNC_ARM]);
}
void moveArmsScoreAsync()
{
  // Start asyncArmController if needed
  startAsyncArmController();
  // Set the proper target
  mutexTake(mutexes[MUTEX_ASYNC_ARM], 1000);
  nextArmTarget = ARM_SCORE;
  isArmAtTarget = false;
  mutexGive(mutexes[MUTEX_ASYNC_ARM]);
}
void moveArmsLowAsync()
{
  // Start asyncArmController if needed
  startAsyncArmController();
  // Set the proper target
  mutexTake(mutexes[MUTEX_ASYNC_ARM], 1000);
  nextArmTarget = ARM_LOW;
  isArmAtTarget = false;
  mutexGive(mutexes[MUTEX_ASYNC_ARM]);
}
void moveArmsMedAsync()
{
  // Start asyncArmController if needed
  startAsyncArmController();
  // Set the proper target
  mutexTake(mutexes[MUTEX_ASYNC_ARM], 1000);
  nextArmTarget = ARM_MED;
  isArmAtTarget = false;
  mutexGive(mutexes[MUTEX_ASYNC_ARM]);
}
int getArmPot()
{
  return analogRead(PORT_armPot);
}
void setArms(int power)
{
  motorSet(PORT_leftArm, power);
  motorSet(PORT_rightArm, -power);
}
void setRollers(int power)
{
  motorSet(PORT_leftRoller, power);
  motorSet(PORT_rightRoller, -power);
}
void stopArms()
{
  motorStop(PORT_leftArm);
  motorStop(PORT_rightArm);
}
void stopRollers()
{
  motorStop(PORT_leftRoller);
  motorStop(PORT_rightRoller);
}
