#include "main.h"

void setArms(int power)
{
  motorSet(PORT_leftArm, power);
  motorSet(PORT_rightArm, -power);
}
void setRollers(int power)
{
  motorSet(PORT_leftRoller, power);
  motorSet(PORT_rightRoller, -power);
}
