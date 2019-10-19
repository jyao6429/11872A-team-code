#include "main.h"

PID armPID;

void moveArmsMed(bool hold)
{
  pidInit(&armPID, 0.05, 0.0, 0.0);

  bool isAtTarget = false;

  while (!isAtTarget)
  {
    // Calculate and set power for arms
    int power = pidCalculate(&armPID, ARM_MED_DIFF, getArmPot());
    setArms(power);

    // Disengages if arm is within 300 ticks, and only if not holding the arm
    isAtTarget = getArmPot() - ARM_ZERO < 300 && !hold;

    delay(20);
  }
  stopArms();
}
void moveArmsLow(bool hold)
{
  pidInit(&armPID, 0.05, 0.0, 0.0);

  bool isAtTarget = false;

  while (!isAtTarget)
  {
    // Calculate and set power for arms
    int power = pidCalculate(&armPID, ARM_LOW_DIFF, getArmPot());
    setArms(power);

    // Disengages if arm is within 300 ticks, and only if not holding the arm
    isAtTarget = getArmPot() - ARM_ZERO < 300 && !hold;

    delay(20);
  }
  stopArms();
}
void moveArmsZero()
{
  pidInit(&armPID, 0.02, 0.0, 0.0);

  bool isAtTarget = false;

  while (!isAtTarget)
  {
    // Calculate and set power for arms
    int power = pidCalculate(&armPID, ARM_ZERO, getArmPot());
    setArms(power);

    // Disengages if arm is within 300 ticks
    isAtTarget = getArmPot() - ARM_ZERO < 300;

    delay(20);
  }
  stopArms();
}
int getArmPot()
{
  return analogRead(PORT_armPot) - ARM_ZERO;
}
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
void stopArms()
{
  motorStop(PORT_leftArm);
  motorStop(PORT_rightArm);
}
void stopRollers()
{
  motorStop(PORT_leftRoller);
  motorStop(PORT_rightRoller);
}
