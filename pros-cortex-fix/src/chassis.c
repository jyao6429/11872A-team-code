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
  PID_KEEP_STRAIGHT
};
// Array of PID controllers for various motions
PID controllers[3];

void driveStraightToPose(double targetX, double targetY, double targetAngle, int maxSpeed, bool isDegrees)
{
  // First drive to target point
  driveStraightToPoint(targetX, targetY, maxSpeed, true);
  // Turn to the correct orientation
  turnToAngle(targetAngle, maxSpeed, true, isDegrees);
}
void driveStraightToPoint(double targetX, double targetY, int maxSpeed, bool isAccurate)
{
  // Initial turn
  turnToAngle(angleToFacePoint(targetX, targetY), maxSpeed, false, false);

  // Initialize PIDs to go and keep straight
  pidInit(&controllers[PID_STRAIGHT], 0.15, 0.0, 0.1);
  pidInit(&controllers[PID_KEEP_STRAIGHT], 2.0, 0.0, 0.2);

  // Variables to keep track of current state
  bool isAtTarget = false;
  unsigned long atTargetTime = millis();

  while (!isAtTarget)
  {
    mutexTake(mutexes[MUTEX_POSE], -1);
    // Get current state
    double distanceToGo = distanceToPoint(targetX, targetY);
    double currentAngle = robotPose[POSE_ANGLE];

    // Get info needed to face in the direction of the point
    double angleToFace = angleToFacePoint(targetX, targetY);
    int robotDirection = 1;
    mutexGive(mutexes[MUTEX_POSE]);

    // Reverses direction instead of turning completely around in case robot overshoots target
    if (fabs(angleToFace - currentAngle) > M_PI / 2)
    {
      angleToFace = nearestEquivalentAngle(angleToFace - M_PI);
      robotDirection = -1;
    }

    /*
    // Compensate for angle error only, with greater compensation when not in margin
    int driveDiff = 0;
    if (distanceToGo > ACCURATE_DISTANCE_ERROR * 2 || !isAccurate)
    {
      // If large error in angle (30 degrees off), turn to face it
      if (fabs(angleToFace - currentAngle) > M_PI / 6)
      {
        turnToAngle(angleToFace, maxSpeed, false, false);
      }
      // Otherwise, give differential power to wheels
      else
      {
        controllers[PID_KEEP_STRAIGHT].Kd = 0.2;
        driveDiff = pidCalculate(&controllers[PID_KEEP_STRAIGHT], angleToFace, robotPose[POSE_ANGLE]) * maxSpeed * 2;
      }
    }
    else if (distanceToGo > ACCURATE_DISTANCE_ERROR)
    {
      // If large error in angle (30 degrees off), turn to face it
      if (fabs(angleToFace - currentAngle) > M_PI / 6)
      {
        turnToAngle(angleToFace, maxSpeed * 0.6, true, false);
      }
      // Otherwise, give differential power to wheels
      else
      {
        controllers[PID_KEEP_STRAIGHT].Kd = 0.5;
        driveDiff = pidCalculate(&controllers[PID_KEEP_STRAIGHT], angleToFace, robotPose[POSE_ANGLE]) * maxSpeed * 2;
      }
    }
    */

    // Calculate the velocity and angular velocity to approach the point
    double inputVelocity = robotDirection * pidCalculate(&controllers[PID_STRAIGHT], distanceToGo, 0) * maxSpeed;
    double inputOmega = pidCalculate(&controllers[PID_KEEP_STRAIGHT], angleToFace, currentAngle) * maxSpeed;

    // Calculate angular velocities of each wheel
    int leftWheelVelocity = (2 * inputVelocity + inputOmega * (sL + sR)) / sideWheelDiameter;
    int rightWheelVelocity = (2 * inputVelocity - inputOmega * (sL + sR)) / sideWheelDiameter;

    // Drive the wheels
    powerMotors(leftWheelVelocity, rightWheelVelocity);

    // Debug
    printf("(driveStraightToPoint) X: %f\tY: %f\tANGLE: %f\n", robotPose[POSE_X], robotPose[POSE_Y], radToDeg(robotPose[POSE_ANGLE]));

    // Checks if at target, depending on accuracy
    if (isAccurate)
    {
      // If not within range, reset timer
      if (distanceToGo > ACCURATE_DISTANCE_ERROR)
      {
        atTargetTime = millis();
      }
      // If within range for at least 350ms, then target is reached
      if (millis() - atTargetTime > 350)
      {
        isAtTarget = true;
        powerMotors(0, 0);
      }
    }
    else
    {
      // If within range, disengage
      if (distanceToGo < INACCURATE_ANGLE_ERROR)
      {
        isAtTarget = true;
      }
    }

    delay(20);
  }
}
void turnToAngle(double targetAngle, int maxSpeed, bool isAccurate, bool isDegrees)
{
  // Convert targetAngle into radians (if needed) and find nearest angle
  if (isDegrees)
    targetAngle = degToRad(targetAngle);

  targetAngle = nearestEquivalentAngle(targetAngle);

  // Initialize turning PID, with larger constants when small angle and slow
  if (maxSpeed < 60 && fabs(targetAngle - robotPose[POSE_ANGLE]) < M_PI / 18)
  {
    pidInit(&controllers[PID_ROTATE], 6.0, 1.0, 0.5);
  }
  else
  {
    pidInit(&controllers[PID_ROTATE], 3.5, 0.2, 0.35);
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
    int driveOut = pidCalculate(&controllers[PID_ROTATE], targetAngle, currentAngle) * maxSpeed;

    // Drive the wheels
    powerMotors(driveOut, -driveOut);

    // Debug
    printf("turnToAngle: %f\tX: %f\tY: %f\tANGLE: %f\n", radToDeg(targetAngle), robotPose[POSE_X], robotPose[POSE_Y], radToDeg(robotPose[POSE_ANGLE]));

    delay(20);
  }
}
