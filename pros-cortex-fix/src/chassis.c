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

void powerMotors(int leftPower, int rightPower)
{
  motorSet(PORT_leftBackMotor, leftPower);
  motorSet(PORT_leftFrontMotor, leftPower);
  motorSet(PORT_rightBackMotor, -rightPower);
  motorSet(PORT_rightFrontMotor, -rightPower);
}
void powerMotorsLinear(int leftPower, int rightPower)
{
  // Clamping fucntion, as well as looking up proper speed in the array
  leftPower = ((leftPower < 0) ? -1 : 1) * ((abs(leftPower) > 127) ? 127 : TrueSpeed[abs(leftPower)]);
  rightPower = ((rightPower < 0) ? -1 : 1) * ((abs(rightPower) > 127) ? 127 : TrueSpeed[abs(rightPower)]);
  powerMotors(leftPower, rightPower);
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

/*
void driveAndParkToPose(double targetX, double targetY, double targetAngle, int maxSpeed, bool isAccurate, bool isDegrees)
{
  if (isDegrees)
    targetAngle = degToRad(targetAngle);

  // Calculate coordinates to line up for parking (12 in away from target point)
  double lineUpX = targetX - 12 * sin(targetAngle);
  double lineUpY = targetY - 12 * cos(targetAngle);

  // Drive to the line up
  driveToPose(lineUpX, lineUpY, targetAngle, maxSpeed, false, false);
  // Park the robot following line
  driveAlongLineToPose(targetX, targetY, targetAngle, maxSpeed, isAccurate, false);
}
void driveAlongLineToPose(double targetX, double targetY, double targetAngle, int maxSpeed, bool isAccurate, bool isDegrees)
{
  // Get the line to follow
  LineTarget targetLine;
  matchLineWithPose(&targetLine, targetX, targetY, targetAngle, isDegrees);

  // If angle is in 2nd or 3rd quadrant, need to reverse the direction the robot turns to meet the line
  int turnDirection = 1;
  targetAngle = normalizeAngle(targetLine.targetAngle);

  if (targetAngle < 0)
    turnDirection = -1;

  // Initialize PIDs
  pidInit(&controllers[PID_ZERO], 0.0, 0.0, 0.0);
  pidInit(&controllers[PID_STRAIGHT], 0.12, 0.0, 0.02);
  pidInit(&controllers[PID_ROTATE_ON_POINT], 0.5, 0.0, 0.05);
  pidInit(&controllers[PID_ROTATE], 0.6, 0.0, 0.0);
  pidInit(&controllers[PID_STAY_ON_TARGET], 0.05, 0.0, 0.0);

  // Actual controllers for rotation
  PID *stayOnTargetAngle = &controllers[PID_ROTATE];
  PID *stayOnLine = &controllers[PID_STAY_ON_TARGET];

  // Variables to keep track of current state
  bool isAtTarget = false;
  unsigned long atTargetTime = millis();

  while (!isAtTarget)
  {
    mutexTake(mutexes[MUTEX_POSE], -1);
    // Get current state
    double distanceToGo = distanceToPointFromRobot(targetX, targetY);
    double distanceFromLine = distanceToLineFromRobot(&targetLine);
    double currentAngle = robotPose[POSE_ANGLE];

    // Get info needed to face in the direction of the point, just to change robot direction
    double angleToFace = angleToFacePointFromRobot(targetX, targetY);
    int robotDirection = 1;

    // Info on targetAngle just in case an entire rotation was made at some point
    targetAngle = nearestEquivalentAngleFromRobot(targetAngle);
    mutexGive(mutexes[MUTEX_POSE]);

    // Reverses direction instead of turning completely around in case robot overshoots target
    if (fabs(angleToFace - currentAngle) > M_PI / 2)
    {
      robotDirection = -1;
    }

    // Calculate the velocity and angular velocity to approach the pose
    double inputVelocity = robotDirection * pidCalculate(&controllers[PID_STRAIGHT], distanceToGo, 0) * maxSpeed * sideWheelDiameter / 2 - robotDirection * 10;

    // Ignore deviation from line if within margin of error
    if (distanceToGo < ACCURATE_DISTANCE_ERROR)
    {
      stayOnLine = &controllers[PID_ZERO];
      stayOnTargetAngle = &controllers[PID_ROTATE_ON_POINT];
    }
    else
    {
      stayOnLine = &controllers[PID_STAY_ON_TARGET];
      stayOnTargetAngle = &controllers[PID_ROTATE];
    }
    // Finally, calculate target angular velocity of the robot
    double stayOnLineCorrection = robotDirection * turnDirection * pidCalculate(stayOnLine, distanceFromLine, 0) * maxSpeed * sideWheelDiameter / 2;
    double stayOnTargetAngleCorrection = pidCalculate(stayOnTargetAngle, targetAngle, currentAngle) * maxSpeed * sideWheelDiameter / 2;

    double inputOmega = stayOnLineCorrection + stayOnTargetAngleCorrection;

    // Calculate angular velocities of each wheel
    int leftWheelVelocity = (2 * inputVelocity + inputOmega * (sL + sR)) / sideWheelDiameter;
    int rightWheelVelocity = (2 * inputVelocity - inputOmega * (sL + sR)) / sideWheelDiameter;

    // Drive the wheels
    powerMotors(leftWheelVelocity, rightWheelVelocity);

    // Debug
    printf("(dAL)iV: %3.3f   iW: %3.3f   DTG: %3.3f   DFL: %3.3f   LC: %3.3f   AC: %3.3f   TA: %3.3f   X: %3.3f   Y: %3.3f   AG: %3.3f\n", inputVelocity, inputOmega, distanceToGo, distanceFromLine, stayOnLineCorrection, stayOnTargetAngleCorrection, radToDeg(targetAngle), robotPose[POSE_X], robotPose[POSE_Y], radToDeg(currentAngle));

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
void driveToPose(double targetX, double targetY, double targetAngle, int maxSpeed, bool isAccurate, bool isDegrees)
{
  // Convert to radians if needed
  if (isDegrees)
    targetAngle = degToRad(targetAngle);

  // Initialize PIDs to drive straight, stay on target, and rotate to the correct orientation
  pidInit(&controllers[PID_ZERO], 0.0, 0.0, 0.0);
  pidInit(&controllers[PID_STRAIGHT], 0.15, 0.0, 0.02);
  pidInit(&controllers[PID_ROTATE_ON_POINT], 0.5, 0.2, 0.05);
  pidInit(&controllers[PID_ROTATE], -0.5, 0.0, 0.0);
  pidInit(&controllers[PID_STAY_ON_TARGET], 1.0, 0.0, 0.0);

  // Also have separate PIDs for the actual motions
  PID *headingPID = &controllers[PID_STAY_ON_TARGET];
  PID *targetAnglePID = &controllers[PID_ROTATE];

  // Variables to keep track of current state
  bool isAtTarget = false;
  unsigned long atTargetTime = millis();

  while (!isAtTarget)
  {
    mutexTake(mutexes[MUTEX_POSE], -1);
    // Get current state
    double distanceToGo = distanceToPointFromRobot(targetX, targetY);
    double currentAngle = normalizeAngle(robotPose[POSE_ANGLE]);

    // Get info needed to face in the direction of the point
    double angleToFace = normalizeAngle(angleToFacePointFromRobot(targetX, targetY));
    int robotDirection = 1;

    // Info on targetAngle just in case an entire rotation was made at some point
    mutexGive(mutexes[MUTEX_POSE]);

    // Reverses direction instead of turning completely around in case robot overshoots target
    if (fabs(angleToFace - currentAngle) > M_PI / 2)
    {
      angleToFace = normalizeAngle(nearestEquivalentAngleFromRobot(angleToFace - M_PI));
      robotDirection = -1;
    }

    // Calculate the velocity and angular velocity to approach the pose
    double inputVelocity = robotDirection  * pidCalculate(&controllers[PID_STRAIGHT], distanceToGo, 0) * maxSpeed * sideWheelDiameter / 2 - robotDirection * 10;

    // If within margin for position, focus on hitting target angle
    if (distanceToGo < INACCURATE_DISTANCE_ERROR)
    {
      headingPID = &controllers[PID_ZERO];
      // If within margin for targetAngle, don't change anything
      if (fabs(targetAngle - currentAngle) < ACCURATE_ANGLE_ERROR)
      {
        targetAnglePID = &controllers[PID_ZERO];
      }
      else
      {
        targetAnglePID = &controllers[PID_ROTATE_ON_POINT];
      }
      angleToFace = currentAngle;
    }
    else
    {
      headingPID = &controllers[PID_STAY_ON_TARGET];
      targetAnglePID = &controllers[PID_ROTATE];
    }

    double headingCorrection = pidCalculate(headingPID, angleToFace, currentAngle) * maxSpeed * sideWheelDiameter / 2;
    double targetAngleCorrection = pidCalculate(targetAnglePID, targetAngle, angleToFace) * maxSpeed * sideWheelDiameter / 2;
    // Finally, calculate target angular velocity of the robot
    double inputOmega = (headingCorrection + targetAngleCorrection);

    // Calculate angular velocities of each wheel
    int leftWheelVelocity = (2 * inputVelocity + inputOmega * (sL + sR)) / sideWheelDiameter;
    int rightWheelVelocity = (2 * inputVelocity - inputOmega * (sL + sR)) / sideWheelDiameter;

    // Drive the wheels
    powerMotors(leftWheelVelocity, rightWheelVelocity);

    // Debug
    printf("(dTP)iV: %3.3f   iW: %3.3f   LW: %d   RW: %d   DTG: %3.3f   HC: %3.3f   TAC: %3.3f   TA: %3.3f   X: %3.3f   Y: %3.3f   AG: %3.3f\n", inputVelocity, inputOmega, leftWheelVelocity, rightWheelVelocity, distanceToGo, headingCorrection, targetAngleCorrection, radToDeg(targetAngle), robotPose[POSE_X], robotPose[POSE_Y], radToDeg(currentAngle));

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
    else
    {
      // If not within range, reset timer
      if (distanceToGo > INACCURATE_DISTANCE_ERROR || fabs(currentAngle - targetAngle) > INACCURATE_ANGLE_ERROR)
      {
        atTargetTime = millis();
      }
      // If within range for at least 100ms, then target is reached
      if (millis() - atTargetTime > 100)
      {
        isAtTarget = true;
      }
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
  turnToAngle(angleToFacePointFromRobot(targetX, targetY), maxSpeed, false, false);

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
    double distanceToGo = distanceToPointFromRobot(targetX, targetY);
    double currentAngle = robotPose[POSE_ANGLE];

    // Get info needed to face in the direction of the point
    double angleToFace = angleToFacePointFromRobot(targetX, targetY);
    int robotDirection = 1;
    mutexGive(mutexes[MUTEX_POSE]);

    // Reverses direction instead of turning completely around in case robot overshoots target
    if (fabs(angleToFace - currentAngle) > M_PI / 2)
    {
      angleToFace = nearestEquivalentAngleFromRobot(angleToFace - M_PI);
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
