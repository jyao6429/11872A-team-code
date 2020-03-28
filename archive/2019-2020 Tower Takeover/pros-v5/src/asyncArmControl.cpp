#include "main.h"

// Devices for arms
Motor armMotor(15);

// The next target position for the arm
int nextArmTarget;

void initArm()
{
  // Set arm motor properties
  armMotor.setGearing(AbstractMotor::gearset::red);
  armMotor.setBrakeMode(AbstractMotor::brakeMode::hold);
  armMotor.setEncoderUnits(AbstractMotor::encoderUnits::degrees);

  // Reset arm controllers
  mutexes[MUTEX_ASYNC_ARM].take(500);
  nextArmTarget = -1;
  mutexes[MUTEX_ASYNC_ARM].give();
  resetArm();
}
void startAsyncArmController()
{
  startAsyncTrayController();
  armMotor.modifyProfiledVelocity(100);
  mutexes[MUTEX_ASYNC_ARM].take(500);
  nextArmTarget = armMotor.getTargetPosition();
  mutexes[MUTEX_ASYNC_ARM].give();
}
void stopAsyncArmController()
{
  armMotor.modifyProfiledVelocity(0);
  mutexes[MUTEX_ASYNC_ARM].take(500);
  nextArmTarget = -1;
  mutexes[MUTEX_ASYNC_ARM].give();
}
bool waitUntilArmMoveComplete(int timeout)
{
  uint32_t timer = pros::millis();
  while (abs(getArmPosition() - nextArmTarget) > 10)
  {
    if (pros::millis() - timer > timeout)
      return true;

    pros::delay(40);
  }
  return false;
}
void waitUntilArmMoveComplete()
{
  while (abs(getArmPosition() - nextArmTarget) > 10) { pros::delay(40); }
}
void moveArmsToPosition(int armTarget)
{
  startAsyncTrayController();
  // Make sure target is within range
  armTarget = (armTarget > ARM_MED + 50) ? ARM_MED + 50 : armTarget;
  armTarget = (armTarget < ARM_ZERO) ? ARM_ZERO: armTarget;

  // Set arm targets
  armMotor.moveAbsolute(armTarget, 100);
  mutexes[MUTEX_ASYNC_ARM].take(500);
  nextArmTarget = armTarget;
  mutexes[MUTEX_ASYNC_ARM].give();
}
void moveArmsZeroAsync() { moveArmsToPosition(ARM_ZERO); }
void moveArmsSecondAsync() { moveArmsToPosition(ARM_SECOND); }
void moveArmsLowAsync() { moveArmsToPosition(ARM_LOW); }
void moveArmsMedAsync() { moveArmsToPosition(ARM_MED); }
void resetArm()
{
  printf("Resetting arms\n");
  setArms(-10);
  pros::delay(250);
  armMotor.tarePosition();
  stopArms();
}
bool needsTrayOverride()
{
  return (nextArmTarget >= 0) && (nextArmTarget == ARM_LOW || nextArmTarget == ARM_MED || (nextArmTarget == ARM_ZERO && abs(getArmPosition() - nextArmTarget) > 200));
}
double getArmPosition()
{
  return armMotor.getPosition();
}
void setArms(double power)
{
  armMotor.moveVoltage(12000 * power);
}
void setArms(int power)
{
  armMotor.moveVoltage((12000.0 * power) / 127.0);
}
void setArmsVel(double speed)
{
  armMotor.moveVelocity(speed);
}
void stopArms()
{
  armMotor.moveVoltage(0);
}
