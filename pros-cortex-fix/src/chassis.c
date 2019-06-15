#include "main.h"

// Accurate to about 1.7 degrees
#define ACCURATE_ANGLE_ERROR 0.03
// Accurate to about 5 degrees
#define INACCURATE_ANGLE_ERROR 0.09
// Accurate to inches
#define ACCURATE_DISTANCE_ERROR 1
#define INACCURATE_DISTANCE_ERROR 4

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

void driveStraightToPoint(double targetX, double targetY, int maxSpeed, bool isAccurate)
{
  // Face in the direction of the point
  turnToAngle(angleToFacePoint(targetX, targetY), maxSpeed, false);

  // Initialize PIDs to go straight and keep pointed at the point
  pidInit(&controllers[PID_STRAIGHT], 0.1, 0.0, 0.0);
  pidInit(&controllers[PID_ROTATE], 0.1, 0.0, 0.0);

  // Variables to keep track of current state
  bool isAtTarget = false;
  unsigned long atTargetTime = millis();

  while (!isAtTarget)
  {
    // Calculate distance to target and differential power
    mutexTake(mutexes[MUTEX_POSE], -1);
    double distanceToGo = distanceToPoint(targetX, targetY);

    // Calculate differences in power to keep facing point
    int driveDiff = pidCalculate(&controllers[PID_ROTATE], angleToFacePoint(targetX, targetY), robotPose[POSE_ANGLE]) * 30;
    mutexGive(mutexes[MUTEX_POSE]);

    // Calculate the speed to approach the point
    int driveOut = -1 * pidCalculate(&controllers[PID_STRAIGHT], 0, distanceToGo) * maxSpeed;

    // Drive the wheels
    powerMotors(driveOut - driveDiff, driveOut + driveDiff);

    // Debug
    printf("Still driving: %f\n", distanceToGo);

    // Calculate if is at target
    if (isAccurate)
    {
      // If not within error, reset timer
      if (distanceToGo > ACCURATE_DISTANCE_ERROR)
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
      if (distanceToGo < INACCURATE_DISTANCE_ERROR)
      {
        isAtTarget = true;
        powerMotors(0, 0);
      }
    }
    delay(20);
  }
}
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
    // Get current angle
    mutexTake(mutexes[MUTEX_POSE], -1);
    double currentAngle = robotPose[POSE_ANGLE];
    mutexGive(mutexes[MUTEX_POSE]);

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
        break;
      }
    }
    else
    {
      // If within range, disengage
      if (fabs(targetAngle - currentAngle) < INACCURATE_ANGLE_ERROR)
      {
        isAtTarget = true;
        break;
      }
    }

    // Calculate power to wheels
    int driveOut = pidCalculate(&controllers[PID_ROTATE], targetAngle, currentAngle) * maxSpeed;

    // Drive the wheels
    powerMotors(driveOut, -driveOut);

    // Debug
    printf("(turnToAngle) X: %f\tY: %f\tANGLE: %f\n", robotPose[POSE_X], robotPose[POSE_Y], radToDeg(robotPose[POSE_ANGLE]));

    delay(20);
  }
}
