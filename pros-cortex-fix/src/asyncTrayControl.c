#include "main.h"

PID trayPID;

void asyncTrayTask(void *ignore)
{
  int prevTrayTarget = -1;
  pidInit(&trayPID, 0.0007, 0.0, 0.0);

  while (true)
  {
    // Precaution in case mutex is taken
    int currentTrayTarget = prevTrayTarget;
    int currentTrayPot = getTrayPot();

    // Take mutexes and set proper variables
    mutexTake(mutexes[MUTEX_ASYNC_TRAY], 200);
    currentTrayTarget = nextTrayTarget;
    mutexGive(mutexes[MUTEX_ASYNC_TRAY]);

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
  // Only if task is not already running
  unsigned int asyncState = taskGetState(asyncTrayHandle);
  if (asyncTrayHandle == NULL || (asyncState == TASK_DEAD))
  {
    // Reset variables
    mutexTake(mutexes[MUTEX_ASYNC_TRAY], 200);
    nextTrayTarget = -1;
    isTrayAtTarget = true;
    mutexGive(mutexes[MUTEX_ASYNC_TRAY]);

    // Stop the tray
    stopTray();

    // Create the task
    asyncTrayHandle = taskCreate(asyncTrayTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT + 1);
  }
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
