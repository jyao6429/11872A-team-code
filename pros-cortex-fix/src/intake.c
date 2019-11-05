#include "main.h"

PID armPID;

void moveArmsMed(bool hold)
{
  pidInit(&armPID, 0.01, 0.002, 0.0);
  bool isAtTarget = false;

  print("Starting moveArmsMed\n");

  while (!isAtTarget)
  {
    // Calculate and set power for arms
    int power = pidCalculate(&armPID, ARM_MED, getArmPot()) * 127 + 20;
    if (power < 0)
      power = 0;
    setArms(power);

    // Debug
    printf("armPot: %d\tpower: %d\ttarget: %d\n", getArmPot(), power, ARM_MED);

    // Disengages if arm is within 50 ticks, and only if not holding the arm
    isAtTarget = abs(getArmPot() - ARM_MED) < 20 && !hold;

    delay(20);
  }
  stopArms();
}
void moveArmsLow(bool hold)
{
  pidInit(&armPID, 0.01, 0.002, 0.0);
  bool isAtTarget = false;

  print("Starting moveArmsLow\n");

  while (!isAtTarget)
  {
    // Calculate and set power for arms
    int power = pidCalculate(&armPID, ARM_LOW, getArmPot()) * 127 + 20;
    if (power < 0)
      power = 0;
    setArms(power);

    // Debug
    printf("armPot: %d\tpower: %d\ttarget: %d\n", getArmPot(), power, ARM_LOW);

    // Disengages if arm is within 50 ticks, and only if not holding the arm
    isAtTarget = abs(getArmPot() - ARM_LOW) < 20 && !hold;

    delay(20);
  }
  stopArms();
}
void moveArmsScore(bool hold)
{
  pidInit(&armPID, 0.01, 0.002, 0.0);
  bool isAtTarget = false;

  print("Starting moveArmsScore\n");

  while (!isAtTarget)
  {
    // Calculate and set power for arms
    int power = pidCalculate(&armPID, ARM_SCORE, getArmPot()) * 127 + 20;
    if (power < 0)
      power = 0;
    setArms(power);

    // Debug
    //printf("armPot: %d\tpower: %d\ttarget: %d\n", getArmPot(), power, ARM_SCORE_DIFF);

    // Disengages if arm is within 50 ticks, and only if not holding the arm
    isAtTarget = abs(getArmPot() - ARM_SCORE) < 50 && !hold;

    delay(20);
  }
  stopArms();
}
void moveArmsZero()
{
  pidInit(&armPID, 0.005, 0.001, 0.0);
  bool isAtTarget = false;

  print("Starting moveArmsZero\n");

  while (!isAtTarget)
  {
    // Calculate and set power for arms
    int power = pidCalculate(&armPID, ARM_ZERO, getArmPot()) * 20 + 20;
    setArms(power);

    // Debug
    printf("armPot: %d\tpower: %d\ttarget: %d\n", getArmPot(), power, ARM_ZERO);

    // Disengages if arm is within 50 ticks
    isAtTarget = getArmPot() < 100;

    delay(20);
  }
  stopArms();
}
int getArmPot()
{
  return analogRead(PORT_armPot);
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
