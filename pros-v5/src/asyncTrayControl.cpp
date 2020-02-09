#include "main.h"

// Handles the async task
std::unique_ptr<pros::Task> asyncTrayHandle;

Motor trayMotor(8);
Potentiometer trayPot('G');

void asyncTrayTask(void *ignore)
{
  bool isTrayVertical = false;
  int prevTrayTarget = -1;

  auto trayStackingController = IterativeControllerFactory::posPID(0.1, 0.0, 0.0);
  auto trayController = IterativeControllerFactory::posPID(0.2, 0.01, 0.01);

  trayStackingController.setOutputLimits(100, -100);
  trayController.setOutputLimits(100, -100);

  trayStackingController.setTarget(TRAY_VERTICAL);
  trayController.setTarget(TRAY_ANGLED);

  while (true)
  {
    // Precaution in case mutex is taken
    int currentTrayTarget = prevTrayTarget;
    int currentTrayPot = getTrayPot();

    mutexes[MUTEX_ASYNC_TRAY].take(200);
    currentTrayTarget = nextTrayTarget;
    mutexes[MUTEX_ASYNC_TRAY].give();

    if (prevTrayTarget == TRAY_ARM)
      currentTrayTarget = TRAY_ANGLED;

    mutexes[MUTEX_ASYNC_TRAY].take(500);
    if (needsTrayOverride())
      currentTrayTarget = TRAY_ARM;
    mutexes[MUTEX_ASYNC_TRAY].give();

    // Disengage if no target set
    if (currentTrayTarget < 0)
      continue;

    if (currentTrayTarget != prevTrayTarget)
      trayController.setTarget(currentTrayTarget);

    double speed = 0;

    if (currentTrayTarget == TRAY_VERTICAL)
      speed = trayStackingController.step(currentTrayPot);
    else
      speed = trayController.step(currentTrayPot);

    setTrayVel(speed);

    // Debug
    printf("trayPot: %d\tspeed: %3.3f\ttarget: %d\n", currentTrayPot, speed, currentTrayTarget);

    // Set prev variables
    prevTrayTarget = currentTrayTarget;

    // Sets if tray is within target
    if (abs(currentTrayTarget - currentTrayPot) < 20)
    {
      mutexes[MUTEX_ASYNC_TRAY].take(200);
      isTrayAtTarget = true;
      mutexes[MUTEX_ASYNC_TRAY].give();
    }
    else
    {
      mutexes[MUTEX_ASYNC_TRAY].take(200);
      isTrayAtTarget = false;
      mutexes[MUTEX_ASYNC_TRAY].give();
    }
    pros::delay(10);
  }
}
void waitUntilTrayMoveComplete()
{
  while (!isTrayAtTarget) { pros::delay(40); }
}
void startAsyncTrayController()
{
  printf("~~~~~~~~~~~~~~Begin startAsyncTrayController~~~~~~~~~~~~~~~~\n");
  // Only if task is not already running
  if (asyncTrayHandle != NULL && (asyncTrayHandle->get_state() != pros::E_TASK_STATE_DELETED) && (asyncTrayHandle->get_state() != pros::E_TASK_STATE_INVALID))
    return;

  // Reset variables
  mutexes[MUTEX_ASYNC_TRAY].take(500);
  nextTrayTarget = -1;
  isTrayAtTarget = true;

  // Stop the tray
  stopTray();

  // Create the task
  asyncTrayHandle = std::make_unique<pros::Task>(asyncTrayTask, nullptr, TASK_PRIORITY_DEFAULT + 1);
  mutexes[MUTEX_ASYNC_TRAY].give();
  printf("~~~~~~~~~~~~~~Finished startAsyncTrayController~~~~~~~~~~~~~~~\n");
}
void stopAsyncTrayController()
{
  // Stop task if needed
  if (asyncTrayHandle != NULL && (asyncTrayHandle->get_state() != pros::E_TASK_STATE_DELETED) && (asyncTrayHandle->get_state() != pros::E_TASK_STATE_INVALID))
    asyncTrayHandle->remove();

  // Reset variables
  mutexes[MUTEX_ASYNC_TRAY].take(500);
  nextTrayTarget = -1;
  isTrayAtTarget = true;
  mutexes[MUTEX_ASYNC_TRAY].give();

  // Stop the tray
  stopTray();
}
void pauseAsyncTrayController()
{
  // Start asyncTrayController if needed
  startAsyncTrayController();
  // Remove next target
  mutexes[MUTEX_ASYNC_TRAY].take(1000);
  nextTrayTarget = -1;
  isTrayAtTarget = true;
  mutexes[MUTEX_ASYNC_TRAY].give();
}
void moveTrayVerticalAsync()
{
  // Start asyncTrayController if needed
  startAsyncTrayController();
  // Set the proper target
  mutexes[MUTEX_ASYNC_TRAY].take(500);
  nextTrayTarget = TRAY_VERTICAL;
  isTrayAtTarget = false;
  mutexes[MUTEX_ASYNC_TRAY].give();
}
void moveTrayAngledAsync()
{
  // Start asyncTrayController if needed
  startAsyncTrayController();
  // Set the proper target
  mutexes[MUTEX_ASYNC_TRAY].take(500);
  nextTrayTarget = TRAY_ANGLED;
  isTrayAtTarget = false;
  mutexes[MUTEX_ASYNC_TRAY].give();
}
void initTray()
{
  trayMotor.setGearing(AbstractMotor::gearset::red);
  startAsyncTrayController();
}
int getTrayPot()
{
  return trayPot.get();
}
void setTray(double power)
{
  trayMotor.moveVoltage(12000 * power);
}
void setTray(int power)
{
  trayMotor.moveVoltage((12000.0 * power) / 127.0);
}
void setTrayVel(double speed)
{
  trayMotor.moveVelocity(speed);
}
void stopTray()
{
  trayMotor.moveVoltage(0);
}
