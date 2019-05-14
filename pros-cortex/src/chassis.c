#include "main.h"
#include "global.h"
#include "chassis.h"

void powerMotors(int leftPower, int rightPower)
{
  motorSet(PORT_leftBackMotor, leftPower);
  motorSet(PORT_leftFrontMotor, leftPower);
  motorSet(PORT_rightBackMotor, rightPower);
  motorSet(PORT_rightFrontMotor, rightPower);
}
