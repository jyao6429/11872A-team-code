#include "main.h"

void asyncArmTask(void *ignore)
{
  // Store the current move
  mutexTake(mutexes[MUTEX_ASYNC_ARM], -1);
  AsyncArmOptions currentArmMove = nextArmMove;
  mutexGive(mutexes[MUTEX_ASYNC_ARM]);

  // Switch between each motion type
  switch (currentArmMove)
  {
    case ASYNC_ARM_ZERO:
      moveArmsZero();
      break;
    case ASYNC_ARM_LOW:
      moveArmsLow(true);
      break;
    case ASYNC_ARM_MED:
      moveArmsMed(true);
      break;
    case ASYNC_ARM_NONE:
      break;
  }
  // Reset variables
  mutexTake(mutexes[MUTEX_ASYNC_ARM], -1);
  nextArmMove = ASYNC_ARM_NONE;
  isArmMoving = false;
  mutexGive(mutexes[MUTEX_ASYNC_ARM]);
}
void waitUntilArmMoveComplete()
{
  while (isArmMoving) { delay(40); }
}
void queueAsyncArmController(AsyncArmOptions moveToQueue)
{
  // Stop task if needed
  unsigned int asyncState = taskGetState(asyncArmHandle);
  if (asyncArmHandle != NULL && (asyncState != TASK_DEAD))
    taskDelete(asyncArmHandle);
  stopArms();

  // Checks if there is an actual motion to do
  if (moveToQueue == ASYNC_ARM_NONE)
    return;

  // Start the task
  mutexTake(mutexes[MUTEX_ASYNC_ARM], -1);
  nextArmMove = moveToQueue;
  isArmMoving = true;
  asyncArmHandle = taskCreate(asyncArmTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT + 1);
  mutexGive(mutexes[MUTEX_ASYNC_ARM]);
}
void stopAsyncArmController()
{
  // Stop task if needed
  unsigned int asyncState = taskGetState(asyncArmHandle);
  if (asyncArmHandle != NULL && (asyncState != TASK_DEAD))
    taskDelete(asyncArmHandle);
  stopArms();

  // Reset variables
  mutexTake(mutexes[MUTEX_ASYNC_ARM], -1);
  nextArmMove = ASYNC_ARM_NONE;
  isArmMoving = false;
  mutexGive(mutexes[MUTEX_ASYNC_ARM]);
}
void moveArmsZeroAsync()
{
  // Queue the next move
  queueAsyncArmController(ASYNC_ARM_ZERO);
}
void moveArmsLowAsync()
{
  // Queue the next move
  queueAsyncArmController(ASYNC_ARM_LOW);
}
void moveArmsMedAsync()
{
  // Queue the next move
  queueAsyncArmController(ASYNC_ARM_MED);
}
