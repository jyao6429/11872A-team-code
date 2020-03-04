#include "main.h"

// Devices for tray
Motor trayMotor(5);
Potentiometer trayPot('G');

// Variables for handling async tasks
// Handles the async task
std::unique_ptr<pros::Task> asyncTrayHandle;
// Keeps track if the tray is at the target
bool isTrayAtTarget;
// The next target pot value for the tray
int nextTrayTarget;

void initTray()
{
  trayMotor.setGearing(AbstractMotor::gearset::green);
  startAsyncTrayController();
}
void asyncTrayTask(void *ignore)
{
  printf("asyncTrayTask - created and starting\n");
  int prevTrayTarget = -1;

  auto trayStackingController = IterativeControllerFactory::posPID(0.065, 0.001, 0.0);
  auto trayController = IterativeControllerFactory::posPID(0.005, 0.0002, 0.0001);

  trayStackingController.setOutputLimits(100, -100);

  trayStackingController.setTarget(TRAY_VERTICAL);
  trayController.setTarget(TRAY_ANGLED);

  printf("asyncTrayTask - starting loop\n");
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

    mutexes[MUTEX_ASYNC_ARM].take(500);
    if (needsTrayOverride())
      currentTrayTarget = TRAY_ARM;
    mutexes[MUTEX_ASYNC_ARM].give();

    // Disengage if no target set
    if (currentTrayTarget < 0)
    {
      if (pros::competition::is_autonomous())
        stopTray();
      pros::delay(10);
      continue;
    }

    if (currentTrayTarget != prevTrayTarget)
      trayController.setTarget(currentTrayTarget);

    double speed = 0;

    if (currentTrayTarget == TRAY_VERTICAL)
    {
      speed = trayStackingController.step(currentTrayPot);
      setTrayVel(speed);
    }
    else
    {
      speed = trayController.step(currentTrayPot);
      setTray(speed);
    }

    // Debug
    //printf("trayPot: %d\tspeed: %3.3f\ttarget: %d\n", currentTrayPot, speed, currentTrayTarget);

    // Set prev variables
    prevTrayTarget = currentTrayTarget;

    // Sets if tray is within target
    if (abs(currentTrayTarget - currentTrayPot) < 50)
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
bool waitUntilTrayMoveComplete(int timeout)
{
  uint32_t timer = pros::millis();
  while (!isTrayAtTarget)
  {
    if (pros::millis() - timer > timeout)
      return true;
    pros::delay(40);
  }
  return false;
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

  printf("Need to start\n");

  // Reset variables
  mutexes[MUTEX_ASYNC_TRAY].take(500);
  printf("Took mutex\n");
  nextTrayTarget = -1;
  isTrayAtTarget = true;
  // Stop the tray
  stopTray();
  printf("Stopped tray\n");
// Create the task
  asyncTrayHandle = std::make_unique<pros::Task>(asyncTrayTask, nullptr, TASK_PRIORITY_DEFAULT + 1);
  printf("Started task\n");
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
void moveTrayToPosition(int trayTarget)
{
  // Start asyncTrayController if needed
  startAsyncTrayController();
  // Make sure target is within range
  trayTarget = (trayTarget > TRAY_VERTICAL) ? TRAY_VERTICAL : trayTarget;
  trayTarget = (trayTarget < TRAY_ANGLED) ? TRAY_ANGLED : trayTarget;
  // Set the proper target
  mutexes[MUTEX_ASYNC_TRAY].take(500);
  nextTrayTarget = trayTarget;
  isTrayAtTarget = false;
  mutexes[MUTEX_ASYNC_TRAY].give();
}
void moveTrayVerticalAsync() { moveTrayToPosition(TRAY_VERTICAL); }
void moveTrayAngledAsync() { moveTrayToPosition(TRAY_ANGLED); }
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
