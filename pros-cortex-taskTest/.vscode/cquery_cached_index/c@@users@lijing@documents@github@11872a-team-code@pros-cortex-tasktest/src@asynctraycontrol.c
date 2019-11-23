#include "main.h"

PID trayPID;
bool isTrayVertical = false;
int prevTrayTarget = -1;

// Handles the async task
TaskHandle asyncTrayHandle;

void asyncTrayLoop()
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
			isTrayVertical = !isTrayVertical;;

      if (isTrayVertical)
        currentTrayTarget = TRAY_VERTICAL;
      else
        currentTrayTarget = TRAY_ANGLED;
			delay(250);
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

  if (prevTrayTarget == TRAY_ARM)
    currentTrayTarget = TRAY_ANGLED;

  mutexTake(mutexes[MUTEX_ASYNC_ARM], 500);
  if (trayOverride)
    currentTrayTarget = TRAY_ARM;
  mutexGive(mutexes[MUTEX_ASYNC_ARM]);

  //print("Tray always printing\n");

  // Disengage if no target set
  if (currentTrayTarget < 0)
  {
    stopTray();
    return;
  }

  // Reinitialize PID if needed
  if (currentTrayTarget != prevTrayTarget)
  {
    if (currentTrayTarget == TRAY_VERTICAL)
      pidInit(&trayPID, 0.0005, 0.0001, 0.0);
    else
      pidInit(&trayPID, 0.0015, 0.0001, 0.0);
  }

  // Calculate and set power for tray
  int power = pidCalculate(&trayPID, currentTrayTarget, currentTrayPot) * 127;
  setTray(power);

  // Debug
  printf("trayPot: %d\tpower: %d\ttarget: %d\ttrayPID.sigma: %3.3f\n", currentTrayPot, power, currentTrayTarget, trayPID.sigma);

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
}
void waitUntilTrayMoveComplete()
{
  while (!isTrayAtTarget) { delay(40); }
}
void startAsyncTrayController()
{
  print("~~~~~~~~~~~~~~Begin startAsyncTrayController~~~~~~~~~~~~~~~~\n");
  // Only if task is not already running
  unsigned int asyncTrayState = taskGetState(asyncTrayHandle);
  printf("asyncTrayState: %d\n", asyncTrayState);
  if (asyncTrayHandle != NULL && (asyncTrayState != TASK_DEAD))
    return;

  // Reset variables
  mutexTake(mutexes[MUTEX_ASYNC_TRAY], 500);
  nextTrayTarget = -1;
  isTrayAtTarget = true;

  // Initialize PID
  pidInit(&trayPID, 0.0005, 0.0001, 0.0);

  // Stop the tray
  stopTray();

  // Create the task
  asyncTrayHandle = taskRunLoop(asyncTrayLoop, 20);
  mutexGive(mutexes[MUTEX_ASYNC_TRAY]);
  print("~~~~~~~~~~~~~~Finished startAsyncTrayController~~~~~~~~~~~~~~~\n");
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
