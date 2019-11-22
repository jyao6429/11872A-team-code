#include "main.h"

PID trayPID;
bool isTrayVertical;

void asyncTrayTask(void *ignore)
{
  int prevTrayTarget = -1;
  pidInit(&trayPID, 0.0005, 0.0003, 0.0);
  isTrayVertical = false;

  while (true)
  {
    // Precaution in case mutex is taken
    int currentTrayTarget = prevTrayTarget;
    int currentTrayPot = getTrayPot();

    // Take mutexes and set proper variables, depending on if operatorControl or not
    if (!isAutonomous() && isEnabled() && isMainConnected)
    {
      // Toggle button for angling the tray
  		if (joystickGetDigital(1, 7, JOY_DOWN))
  		{
  			if (isTrayVertical)
  			{
  				moveTrayAngledAsync();
  				isTrayVertical = false;
  			}
  			else
  			{
  				moveTrayVerticalAsync();
  				isTrayVertical = true;
  			}
  			delay(500);
  		}
  		else if (joystickGetDigital(1, 7, JOY_UP))
  		{
        currentTrayTarget = -1;
        prevTrayTarget = currentTrayTarget;
        mutexTake(mutexes[MUTEX_ASYNC_TRAY], 200);
        isTrayAtTarget = true;
        mutexGive(mutexes[MUTEX_ASYNC_TRAY]);
  		}
    }
    else
    {
      mutexTake(mutexes[MUTEX_ASYNC_TRAY], 200);
      currentTrayTarget = nextTrayTarget;
      mutexGive(mutexes[MUTEX_ASYNC_TRAY]);
    }

    mutexTake(mutexes[MUTEX_ASYNC_ARM], 500);
    if (nextArmTarget > 0 && ((nextArmTarget != ARM_ZERO) || (nextArmTarget == ARM_ZERO && !isArmAtTargetTray)))
      currentTrayTarget = TRAY_ARM;
    mutexGive(mutexes[MUTEX_ASYNC_ARM]);

    // Disengage if no target set
    if (currentTrayTarget < 0)
      continue;

    // Calculate and set power for tray
    int power = pidCalculate(&trayPID, currentTrayTarget, currentTrayPot) * 127;
    setTray(power);

    // Debug
    printf("trayPot: %d\tpower: %d\ttarget: %d\n", currentTrayPot, power, currentTrayTarget);

    // Set prev variables
    prevTrayTarget = currentTrayTarget;

    // Sets if tray is within target
    if (abs(currentTrayTarget - currentTrayPot) < 20)
    {
      mutexTake(mutexes[MUTEX_ASYNC_TRAY], 200);
      isTrayAtTarget = true;
      mutexGive(mutexes[MUTEX_ASYNC_TRAY]);
    }
    else
    {
      mutexTake(mutexes[MUTEX_ASYNC_TRAY], 200);
      isTrayAtTarget = false;
      mutexGive(mutexes[MUTEX_ASYNC_TRAY]);
    }

    delay(20);
  }
}
void waitUntilTrayMoveComplete()
{
  while (!isTrayAtTarget) { delay(40); }
}
void startAsyncTrayController()
{
  print("startAsyncTrayController\n");
  // Only if task is not already running
  unsigned int asyncState = taskGetState(asyncTrayHandle);
  if (asyncTrayHandle != NULL && (asyncState != TASK_DEAD))
    return;

  print("Need to start tray\n");
  // Reset variables
  mutexTake(mutexes[MUTEX_ASYNC_TRAY], 200);
  print("Took MUTEX_ASYNC_TRAY\n");
  nextTrayTarget = -1;
  isTrayAtTarget = true;
  mutexGive(mutexes[MUTEX_ASYNC_TRAY]);
  print("Gave MUTEX_ASYNC_TRAY\n");

  // Stop the tray
  stopTray();
  print("Stopped tray\n");

  // Create the task
  print("Creating task\n");
  asyncTrayHandle = taskCreate(asyncTrayTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT + 1);
  print("Started Tray\n");
}
void stopAsyncTrayController()
{
  // Stop task if needed
  unsigned int asyncState = taskGetState(asyncTrayHandle);
  if (asyncTrayHandle != NULL && (asyncState != TASK_DEAD))
    taskDelete(asyncTrayHandle);

  // Reset variables
  mutexTake(mutexes[MUTEX_ASYNC_TRAY], 500);
  nextTrayTarget = -1;
  isTrayAtTarget = true;
  mutexGive(mutexes[MUTEX_ASYNC_TRAY]);

  // Stop the tray
  stopTray();
}
void moveTrayVerticalAsync()
{
  // Start asyncTrayController if needed
  startAsyncTrayController();
  // Set the proper target
  mutexTake(mutexes[MUTEX_ASYNC_TRAY], 1000);
  nextTrayTarget = TRAY_VERTICAL;
  isTrayAtTarget = false;
  mutexGive(mutexes[MUTEX_ASYNC_TRAY]);
}
void moveTrayAngledAsync()
{
  // Start asyncTrayController if needed
  startAsyncTrayController();
  // Set the proper target
  mutexTake(mutexes[MUTEX_ASYNC_TRAY], 1000);
  nextTrayTarget = TRAY_ANGLED;
  isTrayAtTarget = false;
  mutexGive(mutexes[MUTEX_ASYNC_TRAY]);
}
int getTrayPot()
{
  return analogRead(PORT_trayPot);
}
void setTray(int power)
{
  motorSet(PORT_leftTray, power);
  motorSet(PORT_rightTray, -power);
}
void stopTray()
{
  motorStop(PORT_leftTray);
  motorStop(PORT_rightTray);
}
