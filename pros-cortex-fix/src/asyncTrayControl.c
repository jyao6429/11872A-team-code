#include "main.h"

void asyncTrayTask(void *ignore)
{
  // Store the current move
  mutexTake(mutexes[MUTEX_ASYNC_TRAY], -1);
  AsyncTrayOptions currentTrayMove = nextTrayMove;
  mutexGive(mutexes[MUTEX_ASYNC_TRAY]);

  // Switch between each motion type
  switch (currentTrayMove)
  {
    case ASYNC_TRAY_VERTICAL:
      moveTrayVertical();
      break;
    case ASYNC_TRAY_ANGLED:
      moveTrayAngled();
      break;
    case ASYNC_TRAY_NONE:
      break;
  }
  // Reset variables
  mutexTake(mutexes[MUTEX_ASYNC_TRAY], -1);
  nextTrayMove = ASYNC_TRAY_NONE;
  isTrayMoving = false;
  mutexGive(mutexes[MUTEX_ASYNC_TRAY]);
}
void waitUntilTrayMoveComplete()
{
  while (isTrayMoving) { delay(40); }
}
void queueAsyncTrayController(AsyncTrayOptions moveToQueue)
{
  // Stop task if needed
  unsigned int asyncState = taskGetState(asyncTrayHandle);
  if (asyncTrayHandle != NULL && (asyncState != TASK_DEAD))
    taskDelete(asyncTrayHandle);
  stopTray();

  // Checks if there is an actual motion to do
  if (moveToQueue == ASYNC_TRAY_NONE)
    return;

  // Start the task
  mutexTake(mutexes[MUTEX_ASYNC_TRAY], -1);
  nextTrayMove = moveToQueue;
  isTrayMoving = true;
  asyncTrayHandle = taskCreate(asyncTrayTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
  mutexGive(mutexes[MUTEX_ASYNC_TRAY]);
}
void stopAsyncTrayController()
{
  // Stop task if needed
  unsigned int asyncState = taskGetState(asyncTrayHandle);
  if (asyncTrayHandle != NULL && (asyncState != TASK_DEAD))
    taskDelete(asyncTrayHandle);

  // Reset variables
  mutexTake(mutexes[MUTEX_ASYNC_TRAY], -1);
  nextTrayMove = ASYNC_TRAY_NONE;
  isTrayMoving = false;
  mutexGive(mutexes[MUTEX_ASYNC_TRAY]);
}
void moveTrayVerticalAsync()
{
  // Queue the next move
  queueAsyncTrayController(ASYNC_TRAY_VERTICAL);
}
void moveTrayAngledAsync()
{
  // Queue the next move
  queueAsyncTrayController(ASYNC_TRAY_ANGLED);
}
