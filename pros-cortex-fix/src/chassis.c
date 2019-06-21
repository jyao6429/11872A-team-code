#include "main.h"

// Accurate to about 2 degrees
#define ACCURATE_ANGLE_ERROR 0.035
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
void stopMotors()
{
  motorStop(PORT_leftBackMotor);
  motorStop(PORT_leftFrontMotor);
  motorStop(PORT_rightBackMotor);
  motorStop(PORT_rightFrontMotor);
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

void driveToPose(double targetX, double targetY, double targetAngle, int maxSpeed, bool isAccurate, bool isDegrees)
{
  // Convert to radians if needed
  if (isDegrees)
    targetAngle = degToRad(targetAngle);

    // Info on targetAngle just in case an entire rotation was made at some point
  targetAngle = nearestEquivalentAngle(targetAngle);

  // Initialize PIDs to drive straight, stay on target, and rotate to the correct orientation
  pidInit(&controllers[PID_ZERO], 0.0, 0.0, 0.0);
  pidInit(&controllers[PID_STRAIGHT], 0.1, 0.0, 0.02);
  pidInit(&controllers[PID_ROTATE_ON_POINT], 0.2, 0.0, 0.05);
  pidInit(&controllers[PID_ROTATE], -0.5, 0.0, 0.0);
  pidInit(&controllers[PID_STAY_ON_TARGET], 0.8, 0.0, 0.0);

  // Also have separate PIDs for the actual motions
  PID headingPID = controllers[PID_STAY_ON_TARGET];
  PID targetAnglePID = controllers[PID_ROTATE];

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

    // Calculate the velocity and angular velocity to approach the pose
    double inputVelocity = robotDirection * pidCalculate(&controllers[PID_STRAIGHT], distanceToGo, 0) * maxSpeed * sideWheelDiameter / 2;

    // If within margin for position, focus on hitting target angle
    if (distanceToGo < ACCURATE_DISTANCE_ERROR * 2)
    {
      headingPID = controllers[PID_ZERO];
      // If within margin for targetAngle, don't change anything
      if (fabs(targetAngle - currentAngle) < ACCURATE_ANGLE_ERROR)
      {
        targetAnglePID = controllers[PID_ZERO];
      }
      else
      {
        targetAnglePID = controllers[PID_ROTATE_ON_POINT];
      }
      angleToFace = currentAngle;
    }
    else
    {
      headingPID = controllers[PID_STAY_ON_TARGET];
      targetAnglePID = controllers[PID_ROTATE];
    }

    // Finally, calculate target angular velocity of the robot
    double inputOmega = (pidCalculate(&headingPID, angleToFace, currentAngle) + pidCalculate(&targetAnglePID, targetAngle, angleToFace)) * maxSpeed * sideWheelDiameter / 2;

    // Calculate angular velocities of each wheel
    int leftWheelVelocity = (2 * inputVelocity + inputOmega * (sL + sR)) / sideWheelDiameter;
    int rightWheelVelocity = (2 * inputVelocity - inputOmega * (sL + sR)) / sideWheelDiameter;

    // Drive the wheels
    powerMotors(leftWheelVelocity, rightWheelVelocity);

    // Debug
    printf("(driveToPose) inputVelocity: %f\t inputOmega: %f\tX: %f\tY: %f\tANGLE: %f\n", inputVelocity, inputOmega, robotPose[POSE_X], robotPose[POSE_Y], radToDeg(robotPose[POSE_ANGLE]));

    if (isAccurate)
    {
      // If not within range, reset timer
      if (distanceToGo > ACCURATE_DISTANCE_ERROR || fabs(currentAngle - targetAngle) > ACCURATE_ANGLE_ERROR)
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
    else if (distanceToGo < INACCURATE_DISTANCE_ERROR && fabs(currentAngle - targetAngle) < INACCURATE_ANGLE_ERROR)
    {
      isAtTarget = true;
    }

    delay(20);
  }
}
void driveStraightToPose(double targetX, double targetY, double targetAngle, int maxSpeed, bool isAccurate, bool isDegrees)
{
  // First drive to target point
  driveStraightToPoint(targetX, targetY, maxSpeed, isAccurate);
  // Turn to the correct orientation
  turnToAngle(targetAngle, maxSpeed, isAccurate, isDegrees);
}
void driveStraightToPoint(double targetX, double targetY, int maxSpeed, bool isAccurate)
{
  // Initial turn
  turnToAngle(angleToFacePoint(targetX, targetY), maxSpeed, false, false);

  // Initialize PIDs to drive straight and stay on target
  pidInit(&controllers[PID_STRAIGHT], 0.15, 0.0, 0.1);
  pidInit(&controllers[PID_STAY_ON_TARGET], 0.2, 0.0, 0.2);

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

    // Calculate the velocity and angular velocity to approach the point
    double inputVelocity = robotDirection * pidCalculate(&controllers[PID_STRAIGHT], distanceToGo, 0) * maxSpeed * sideWheelDiameter / 2;
    double inputOmega = pidCalculate(&controllers[PID_STAY_ON_TARGET], angleToFace, currentAngle) * maxSpeed * sideWheelDiameter / 2;

    // Calculate angular velocities of each wheel
    int leftWheelVelocity = (2 * inputVelocity + inputOmega * (sL + sR)) / sideWheelDiameter;
    int rightWheelVelocity = (2 * inputVelocity - inputOmega * (sL + sR)) / sideWheelDiameter;

    // Drive the wheels
    powerMotors(leftWheelVelocity, rightWheelVelocity);

    // Debug
    printf("(driveStraightToPoint) leftWheelVelocity: %d\tX: %f\tY: %f\tANGLE: %f\n", leftWheelVelocity, robotPose[POSE_X], robotPose[POSE_Y], radToDeg(robotPose[POSE_ANGLE]));

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
    pidInit(&controllers[PID_ROTATE_ON_POINT], 6.0, 1.0, 0.5);
  }
  else if (maxSpeed > 80)
  {
    pidInit(&controllers[PID_ROTATE_ON_POINT], 2.0, 0.1, 0.3);
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
