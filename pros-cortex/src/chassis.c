#include "main.h"
#include "constants.h"
#include "chassis.h"

void powerMotors(int leftPower, int rightPower)
{
  motorSet(leftBackMotor, leftPower);
  motorSet(leftFrontMotor, leftPower);
  motorSet(rightBackMotor, rightPower);
  motorSet(rightFrontMotor, rightPower);
}
