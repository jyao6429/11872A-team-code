#include "main.h"

// Accurate to about 2 degrees
#define ACCURATE_ANGLE_ERROR 0.035
// Accurate to about 5 degrees
#define INACCURATE_ANGLE_ERROR 0.09
// Accurate to inches
#define ACCURATE_DISTANCE_ERROR 1.5
#define INACCURATE_DISTANCE_ERROR 3

// compensates for non-linearity of control value vs speed curve
const unsigned int TrueSpeed[128] =
{
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0, 21, 21, 21, 22, 22, 22, 23, 24, 24,
 25, 25, 25, 25, 26, 27, 27, 28, 28, 28,
 28, 29, 30, 30, 30, 31, 31, 32, 32, 32,
 33, 33, 34, 34, 35, 35, 35, 36, 36, 37,
 37, 37, 37, 38, 38, 39, 39, 39, 40, 40,
 41, 41, 42, 42, 43, 44, 44, 45, 45, 46,
 46, 47, 47, 48, 48, 49, 50, 50, 51, 52,
 52, 53, 54, 55, 56, 57, 57, 58, 59, 60,
 61, 62, 63, 64, 65, 66, 67, 67, 68, 70,
 71, 72, 72, 73, 74, 76, 77, 78, 79, 79,
 80, 81, 83, 84, 84, 86, 86, 87, 87, 88,
 88, 89, 89, 90, 90,127,127,127
};

void setDrive(int leftPower, int rightPower)
{
  motorSet(PORT_leftMotor0, -leftPower);
  motorSet(PORT_leftMotor1, leftPower);
  motorSet(PORT_rightMotor0, rightPower);
  motorSet(PORT_rightMotor1, -rightPower);
}
void setDriveLinear(int leftPower, int rightPower)
{
  // Clamping fucntion, as well as looking up proper speed in the array
  leftPower = ((leftPower < 0) ? -1 : 1) * ((abs(leftPower) > 127) ? 127 : TrueSpeed[abs(leftPower)]);
  rightPower = ((rightPower < 0) ? -1 : 1) * ((abs(rightPower) > 127) ? 127 : TrueSpeed[abs(rightPower)]);
  setDrive(leftPower, rightPower);
}
void stopDrive()
{
  motorStop(PORT_leftMotor0);
  motorStop(PORT_leftMotor1);
  motorStop(PORT_rightMotor0);
  motorStop(PORT_rightMotor1);
}

// Enums for the various PID controllers
enum PIDControllers
{
  PID_ZERO,
  PID_STRAIGHT,
  PID_ROTATE_ON_POINT,
  PID_ROTATE,
  PID_STAY_ON_TARGET
};
// Array of PID controllers for various motions
PID controllers[5];

/*
void turnToAngle(double targetAngle, int maxSpeed, bool isAccurate, bool isDegrees)
{
  // Convert targetAngle into radians (if needed) and find nearest angle
  if (isDegrees)
    targetAngle = degToRad(targetAngle);

  targetAngle = nearestEquivalentAngleFromRobot(targetAngle);

  // Initialize turning PID, with larger constants when small angle and slow
  if (maxSpeed < 60 && fabs(targetAngle - robotPose[POSE_ANGLE]) < M_PI / 18)
  {
    pidInit(&controllers[PID_ROTATE_ON_POINT], 6.0, 1.0, 0.5);
  }
  else if (maxSpeed > 80)
  {
    pidInit(&controllers[PID_ROTATE_ON_POINT], 1.5, 0.0, 0.0);
  }
  else
  {
    pidInit(&controllers[PID_ROTATE_ON_POINT], 3.5, 0.2, 0.35);
  }

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
      // If at target for more than 350 milliseconds, disengage
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
    int driveOut = pidCalculate(&controllers[PID_ROTATE_ON_POINT], targetAngle, currentAngle) * maxSpeed;

    // Drive the wheels
    powerMotors(driveOut, -driveOut);

    // Debug
    printf("turnToAngle: %f\tX: %f\tY: %f\tANGLE: %f\n", radToDeg(targetAngle), robotPose[POSE_X], robotPose[POSE_Y], radToDeg(robotPose[POSE_ANGLE]));

    delay(20);
  }
}
*/
