#include "main.h"

void setDrive(double leftPower, double rightPower)
{
  chassis->getModel()->tank(leftPower, rightPower);
}
void setDrive(int leftPower, int rightPower)
{
  chassis->getModel()->tank((double) leftPower / 127.0, (double) rightPower / 127.0);
}
void setDriveVel(double leftSpeed, double rightSpeed)
{
  chassis->getModel()->left(leftSpeed);
  chassis->getModel()->right(rightSpeed);
}
void stopDrive()
{
  chassis->getModel()->tank(0, 0);
}
