#include "main.h"

#define ONE_CUBE_DEGREES 120

Motor leftIntakeMotor(1);
Motor rightIntakeMotor(-11);

void initRollers()
{
    // Set intake motor properties
    leftIntakeMotor.setGearing(AbstractMotor::gearset::red);
    leftIntakeMotor.setBrakeMode(AbstractMotor::brakeMode::brake);
    leftIntakeMotor.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    rightIntakeMotor.setGearing(AbstractMotor::gearset::red);
    rightIntakeMotor.setBrakeMode(AbstractMotor::brakeMode::brake);
    rightIntakeMotor.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
}

bool waitUntilRollerMoveComplete(int timeout)
{
  uint32_t timer = pros::millis();

  while (!leftIntakeMotor.isStopped() || !rightIntakeMotor.isStopped())
  {
    if (pros::millis() - timer > timeout)
      return true;
    pros::delay(40);
  }
  return false;
}
void outtakeOneCubeAsync()
{
  moveRollersRelative(-ONE_CUBE_DEGREES, 127);
}

void moveRollersRelative(int degrees, double velocity)
{
  leftIntakeMotor.moveRelative(degrees, velocity);
  rightIntakeMotor.moveRelative(degrees, velocity);
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
void stopRollers()
{
  leftIntakeMotor.moveVoltage(0);
  rightIntakeMotor.moveVoltage(0);
}
