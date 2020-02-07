#include "main.h"

void waitUntilArmMoveComplete()
{
  while (!isArmAtTarget) { delay(40); }
}
void moveArmsZeroAsync()
{

}
void moveArmsScoreAsync()
{

}
void moveArmsLowAsync()
{

}
void moveArmsMedAsync()
{

}
int getArmPot()
{
  return trayPot.get();
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
  leftIntakeMotor.moveVoltage(12000 * -power);
  rightIntakeMotor.moveVoltage(12000 * power);
}
void setRollers(int power)
{
  leftIntakeMotor.moveVoltage((12000.0 * -power) / 127.0));
  rightIntakeMotor.moveVoltage((12000.0 * power) / 127.0));
}
void setRollersVel(double speed)
{
  leftIntakeMotor.moveVelocity(-speed);
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
