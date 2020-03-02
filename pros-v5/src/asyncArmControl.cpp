#include "main.h"

// Devices for arms
Motor armMotor(15);
Motor leftIntakeMotor(1);
Motor rightIntakeMotor(-11);

// The next target position for the arm
int nextArmTarget;

void initArm()
{
  // Set arm motor properties
  armMotor.setGearing(AbstractMotor::gearset::green);
  armMotor.setBrakeMode(AbstractMotor::brakeMode::hold);
  armMotor.setEncoderUnits(AbstractMotor::encoderUnits::degrees);

  // Reset arm controllers
  mutexes[MUTEX_ASYNC_ARM].take(500);
  nextArmTarget = -1;
  mutexes[MUTEX_ASYNC_ARM].give();
  resetArm();

  // Set intake motor properties
  leftIntakeMotor.setGearing(AbstractMotor::gearset::red);
  leftIntakeMotor.setBrakeMode(AbstractMotor::brakeMode::brake);
  rightIntakeMotor.setGearing(AbstractMotor::gearset::red);
  rightIntakeMotor.setBrakeMode(AbstractMotor::brakeMode::brake);
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
void waitUntilArmMoveComplete()
{
  while (abs(getArmPosition() - nextArmTarget) > 50) { pros::delay(40); }
}
void moveArmsToPosition(int armTarget)
{
  startAsyncTrayController();
  // Make sure target is within range
  armTarget = (armTarget > ARM_MED + 500) ? ARM_MED + 500 : armTarget;
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
  return (nextArmTarget >= 0) && (nextArmTarget != ARM_ZERO || (nextArmTarget == ARM_ZERO && abs(getArmPosition() - nextArmTarget) > 1000));
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
void setRollers(double power)
{
  leftIntakeMotor.moveVoltage(12000 * power);
  rightIntakeMotor.moveVoltage(12000 * power);
}
void setRollers(int power)
{
  leftIntakeMotor.moveVoltage((12000.0 * power) / 127.0);
  rightIntakeMotor.moveVoltage((12000.0 * power) / 127.0);
}
void setRollersVel(double speed)
{
  leftIntakeMotor.moveVelocity(speed);
  rightIntakeMotor.moveVelocity(speed);
}
void stopArms()
{
  armMotor.moveVoltage(0);
}
void stopRollers()
{
  leftIntakeMotor.moveVoltage(0);
  rightIntakeMotor.moveVoltage(0);
}
