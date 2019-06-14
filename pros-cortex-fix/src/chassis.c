#include "main.h"

// Accurate to about 1.7 degrees
#define ACCURATE_ANGLE_ERROR 0.03
// Accurate to about 10 degrees
#define INACCURATE_ANGLE_ERROR 0.17

void powerMotors(int leftPower, int rightPower)
{
  motorSet(PORT_leftBackMotor, leftPower);
  motorSet(PORT_leftFrontMotor, leftPower);
  motorSet(PORT_rightBackMotor, -rightPower);
  motorSet(PORT_rightFrontMotor, -rightPower);
}

// Enums for the various PID controllers
enum PIDControllers
{
  PID_STRAIGHT,
  PID_ROTATE,
};
// Array of PID controllers for various motions
PID controllers[2];

void turnToAngle(double targetAngle, int maxSpeed, bool isAccurate)
{
  // Convert targetAngle into radians and find nearest angle
  targetAngle = nearestEquivalentAngle(degToRad(targetAngle));

  // Initialize turning PID
  pidInit(&controllers[PID_ROTATE], 1.8, 2.0, 0.15);

  // Variables to keep track of current state
  bool isAtTarget = false;
  unsigned long atTargetTime = millis();

  while (!isAtTarget)
  {
    // Calculate the amount to drive to each wheel
    mutexTake(mutexes[MUTEX_POSE], -1);
    double currentAngle = robotPose[POSE_ANGLE];
    mutexGive(mutexes[MUTEX_POSE]);

    int driveOut = pidCalculate(&controllers[PID_ROTATE], targetAngle, currentAngle) * maxSpeed;

    // Drive the wheels
    powerMotors(driveOut, -driveOut);

    // Debug
    printf("Still turning: %f\n", radToDeg(currentAngle));

    // Calculate if is at target
    if (isAccurate)
    {
      // If not within error, reset timer
      if (fabs(targetAngle - currentAngle) > ACCURATE_ANGLE_ERROR)
      {
        atTargetTime = millis();
      }
      // If at target for more than 350 milliseconds
      if (millis() - atTargetTime > 350)
      {
        isAtTarget = true;
        powerMotors(0, 0);
      }
    }
    else
    {
      // If within range, disengage
      if (fabs(targetAngle - currentAngle) < INACCURATE_ANGLE_ERROR)
      {
        isAtTarget = true;
        powerMotors(0, 0);
      }
    }
  }
}
