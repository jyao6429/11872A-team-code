#include "main.h"

Motor armMotor(6);
Motor leftIntakeMotor(7);
Motor rightIntakeMotor(-4);

void initArm()
{
  armMotor.setGearing(AbstractMotor::gearset::red);
  armMotor.setBrakeMode(AbstractMotor::brakeMode::hold);

  nextArmTarget = -1;
  resetArm();

  leftIntakeMotor.setGearing(AbstractMotor::gearset::green);
  leftIntakeMotor.setBrakeMode(AbstractMotor::brakeMode::brake);
  rightIntakeMotor.setGearing(AbstractMotor::gearset::green);
  rightIntakeMotor.setBrakeMode(AbstractMotor::brakeMode::brake);
}
void resetArm()
{
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
void startAsyncArmController()
{
  armMotor.modifyProfiledVelocity(100);
  nextArmTarget = armMotor.getTargetPosition();
}
void stopAsyncArmController()
{
  armMotor.modifyProfiledVelocity(0);
  nextArmTarget = -1;
}
void waitUntilArmMoveComplete()
{
  while (abs(getArmPosition() - nextArmTarget) > 50) { pros::delay(40); }
}
void moveArmsZeroAsync()
{
  armMotor.moveAbsolute(ARM_ZERO, 100);
  nextArmTarget = ARM_ZERO;
}
void moveArmsScoreAsync()
{
  armMotor.moveAbsolute(ARM_SCORE, 100);
  nextArmTarget = ARM_SCORE;
}
void moveArmsLowAsync()
{
  armMotor.moveAbsolute(ARM_LOW, 100);
  nextArmTarget = ARM_LOW;
}
void moveArmsMedAsync()
{
  armMotor.moveAbsolute(ARM_MED, 100);
  nextArmTarget = ARM_MED;
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
