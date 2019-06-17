#include "main.h"

// Physical parameters in inches
const double sL = 4.568;                 // distance from center to left tracking wheel
const double sR = 4.568;                 // distance from center to right tracking wheel
const double sB = 4.5;                   // distance from center to back tracking wheel
const double sideWheelDiameter = 2.75;   // diameter of side wheels
const double backWheelDiameter = 2.75;   // diameter of back wheel
// Encoder counts
const int sideEncoderResolution = 360;   // side encoder ticks per 360 degrees of motion
const int backEncoderResolution = 360;   // back encoder ticks per 360 degrees of motion

// Previous position and orientation
double robotPose[3] = {0.0, 0.0, 0.0};
double resetAngle = 0.0;
// Previous encoder values
int prevLeftEncoder = 0;
int prevRightEncoder = 0;
int prevBackEncoder = 0;

// Not sure if using v5 or cortex, just rewrite these functions to make code work
int getLeftEncoder()
{
  return encoderGet(leftEncoder);
}
int getRightEncoder()
{
  return encoderGet(rightEncoder);
}
int getBackEncoder()
{
  return encoderGet(backEncoder);
}
void resetLeftEncoder()
{
  encoderReset(leftEncoder);
  prevLeftEncoder = 0;
}
void resetRightEncoder()
{
  encoderReset(rightEncoder);
  prevRightEncoder = 0;
}
void resetBackEncoder()
{
  encoderReset(backEncoder);
  prevBackEncoder = 0;
}
void initializeAPS(double startX, double startY, double startAngle)
{
  // Reset to starting positions
  resetPosition(startX, startY, startAngle);

  // Create new tasks to track position
  taskCreate(startTracking, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT + 1);
}
void resetPosition(double resetX, double resetY, double resetA)
{
  // Reset all the encoders
  resetLeftEncoder();
  resetRightEncoder();
  resetBackEncoder();

  // Set reset pose
  mutexTake(mutexes[MUTEX_POSE], -1);
  robotPose[POSE_X] = resetX;
  robotPose[POSE_Y] = resetY;
  robotPose[POSE_ANGLE] = degToRad(resetA);
  mutexGive(mutexes[MUTEX_POSE]);

  // Set reset orientation
  resetAngle = degToRad(resetA);
}
void startTracking(void *ignore)
{
  while (true)
  {
    // Get current encoder values
    int currentLeftEncoder = getLeftEncoder();
    int currentRightEncoder = getRightEncoder();
    int currentBackEncoder = getBackEncoder();

    // Calculate traveled distance in inches
    double deltaLeftDistance = calculateTravelDistance(currentLeftEncoder - prevLeftEncoder, sideWheelDiameter, sideEncoderResolution);
    double deltaRightDistance = calculateTravelDistance(currentRightEncoder - prevRightEncoder, sideWheelDiameter, sideEncoderResolution);
    double deltaBackDistance = calculateTravelDistance(currentBackEncoder - prevBackEncoder, backWheelDiameter, backEncoderResolution);

    // Update prev values;
    prevLeftEncoder = currentLeftEncoder;
    prevRightEncoder = currentRightEncoder;
    prevBackEncoder = currentBackEncoder;

    // Calculate total change since last reset
    double totalLeftDistance = calculateTravelDistance(currentLeftEncoder, sideWheelDiameter, sideEncoderResolution);
    double totalRightDistance = calculateTravelDistance(currentRightEncoder, sideWheelDiameter, sideEncoderResolution);

    // Calculate new absolute orientation
    double newAngle = resetAngle + (totalLeftDistance - totalRightDistance) / (sL + sR);

    // Calculate change in angle
    double deltaAngle = (deltaLeftDistance - deltaRightDistance) / (sL + sR);

    // Calculate local offset vector
    double localOffset[] = {0.0, 0.0};

    // If drove straight
    if (deltaAngle == 0.0)
    {
      localOffset[X_COMP] = deltaBackDistance;
      localOffset[Y_COMP] = deltaRightDistance;
    }
    else
    {
      localOffset[X_COMP] = 2 * sin(deltaAngle / 2) * ((deltaBackDistance / deltaAngle) + sB);
      localOffset[Y_COMP] = 2 * sin(deltaAngle / 2) * ((deltaRightDistance / deltaAngle) + sR);
    }

    // Calculate average angle
    double avgAngle = newAngle - (deltaAngle / 2);

    // Convert localOffset to a polar vector
    double localPolar[] = {0.0, 0.0};
    cartToPolar(localOffset, localPolar);

    // Shift angle
    localPolar[ANGLE] -= avgAngle;

    // Converting back to cartesian gives the globalOffset
    double globalOffset[] = {0.0, 0.0};
    polarToCart(localPolar, globalOffset);

    // Calculate new absolute position and orientation
    mutexTake(mutexes[MUTEX_POSE], -1);
    robotPose[POSE_X] += globalOffset[X_COMP];
    robotPose[POSE_Y] += globalOffset[Y_COMP];
    robotPose[POSE_ANGLE] = newAngle;
    mutexGive(mutexes[MUTEX_POSE]);

    delay(2);
  }
}
double distanceToPoint(double targetX, double targetY)
{
  return sqrt(pow(targetX - robotPose[POSE_X], 2) + pow(targetY - robotPose[POSE_Y], 2));
}
double angleToFacePoint(double targetX, double targetY)
{
  double toReturn = nearestEquivalentAngle(-1 * atan2(targetY - robotPose[POSE_Y], targetX - robotPose[POSE_X]) + M_PI / 2);
  //printf("angleToFacePoint: %f\tX: %f\tY: %f\ttargetX: %f\ttargetY: %f\n", toReturn, robotPose[POSE_X], robotPose[POSE_Y], targetX, targetY);
  return toReturn;
}
double nearestEquivalentAngle(double target)
{
  return round((robotPose[POSE_ANGLE] - target) / (2 * M_PI)) * 2 * M_PI + target;
}
void cartToPolar(double *cartVector, double *polarVector)
{
  // Calculates magnitude of vector with distance formula
  polarVector[MAGNITUDE] = sqrt(pow(cartVector[X_COMP], 2) + pow(cartVector[Y_COMP], 2));
  // Calculates angle with arctan, automatically gives angle in correct quadrant
  polarVector[ANGLE] = atan2(cartVector[Y_COMP], cartVector[X_COMP]);
}
void polarToCart(double *polarVector, double *cartVector)
{
  // Calculate x component with cosine
  cartVector[X_COMP] = polarVector[MAGNITUDE] * cos(polarVector[ANGLE]);
  // Calculate y component with sine
  cartVector[Y_COMP] = polarVector[MAGNITUDE] * sin(polarVector[ANGLE]);
}
double calculateTravelDistance(int encoderCount, double wheelDiameter, int encoderResolution)
{
  return ((double) encoderCount * M_PI * wheelDiameter) / ((double) encoderResolution);
}
double degToRad(double degrees)
{
  return degrees * (M_PI / 180);
}
double radToDeg(double radians)
{
  return radians * (180 / M_PI);
}
