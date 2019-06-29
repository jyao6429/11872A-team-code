#include "main.h"

// distance from center to left tracking wheel
const double sL = 4.568;
// distance from center to right tracking wheel
const double sR = 4.568;
// distance from center to back tracking wheel
const double sB = 4.77;
// diameter of side wheels
const double sideWheelDiameter = 2.75;
// diameter of back wheel
const double backWheelDiameter = 2.75;
// side encoder ticks per 360 degrees of motion
const int sideEncoderResolution = 360;
// back encoder ticks per 360 degrees of motion
const int backEncoderResolution = 360;

void resetPositionFull(Pose *position, double startX, double startY, double startAngle, bool isDegrees)
{
  // Stop task
  taskDelete(APSTask);

  // Reset everything
  resetPosition(position);

  // Reset all the encoders
  resetLeftEncoder();
  resetRightEncoder();
  resetBackEncoder();

  // Convert to radians if needed
  if (isDegrees)
    startAngle = degToRad(startAngle);

  // Set the new positions
  resetAngle = startAngle;
  position->angle = startAngle;
  position->x = startX;
  position->y = startY;

  // Create new task to track position
  APSTask = taskCreate(trackPoseTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT + 2);
}
void resetPosition(Pose *position)
{
  // Reset everything to 0
  position->angle = position->x = position->y = 0.0;
  position->prevLeft = position->prevRight = position->prevBack = 0;
}
void resetVelocity(Vel *velocity, Pose position)
{
  // Reset velocities to 0
  velocity->angle = velocity->x = velocity->y = 0;

  // Set position and time to current
  velocity->prevPoseAngle = position.angle;
  velocity->prevPoseX = position.x;
  velocity->prevPoseY = position.y;

  velocity->prevTime = millis();
}
void trackPosition(Pose *position, int currentLeft, int currentRight, int currentBack)
{
  // Calculate traveled distance in inches
  double deltaLeftDistance = calculateTravelDistance(currentLeft - position->prevLeft, sideWheelDiameter, sideEncoderResolution);
  double deltaRightDistance = calculateTravelDistance(currentRight - position->prevRight, sideWheelDiameter, sideEncoderResolution);
  double deltaBackDistance = calculateTravelDistance(currentBack - position->prevBack, backWheelDiameter, backEncoderResolution);

  // Update prev values;
  position->prevLeft = currentLeft;
  position->prevRight = currentRight;
  position->prevBack = currentBack;

  // Calculate total change since last reset
  double totalLeftDistance = calculateTravelDistance(currentLeft, sideWheelDiameter, sideEncoderResolution);
  double totalRightDistance = calculateTravelDistance(currentRight, sideWheelDiameter, sideEncoderResolution);

  // Calculate new absolute orientation
  double newAngle = resetAngle + (totalLeftDistance - totalRightDistance) / (sL + sR);

  // Calculate change in angle
  double deltaAngle = (deltaLeftDistance - deltaRightDistance) / (sL + sR);

  // Calculate local offset vector
  Cart localOffset;

  // If drove straight
  if (deltaAngle == 0.0)
  {
    localOffset.x = deltaBackDistance;
    localOffset.y = deltaRightDistance;
  }
  else
  {
    localOffset.x = 2.0 * sin(deltaAngle / 2.0) * ((deltaBackDistance / deltaAngle) + sB);
    localOffset.y = 2.0 * sin(deltaAngle / 2.0) * ((deltaRightDistance / deltaAngle) + sR);
  }

  // Calculate average angle
  double avgAngle = newAngle - (deltaAngle / 2);

  // Calculate cosine and sine of avgAngle
  double cosAvg = cos(avgAngle);
  double sinAvg = sin(avgAngle);

  // Update the global position by incrementing by a shifted localOffset
  mutexTake(mutexes[MUTEX_POSE], -1);
  position->x += localOffset.x * cosAvg + localOffset.y * sinAvg;
  position->y += localOffset.x * -sinAvg + localOffset.y * cosAvg;
  position->angle = newAngle;
  mutexGive(mutexes[MUTEX_POSE]);
}
void trackVelocity(Vel *velocity, Pose position)
{
  // Timer for velocity
  unsigned long currentTime = millis();
  long deltaTime = currentTime - velocity->prevTime;

  // Only run every 40 ms, so have time to change position
  if (deltaTime > 40)
  {
    // Get position info
    double poseAngle = position.angle;
    double poseX = position.x;
    double poseY = position.y;

    // Calculate velocities
    velocity->angle = (poseAngle - velocity->prevPoseAngle) / ((double) deltaTime * 0.001);
    velocity->x = (poseX - velocity->prevPoseX) / ((double) deltaTime * 0.001);
    velocity->y = (poseY - velocity->prevPoseY) / ((double) deltaTime * 0.001);

    // Update previous position and time info
    velocity->prevPoseAngle = poseAngle;
    velocity->prevPoseX = poseX;
    velocity->prevPoseY = poseY;
    velocity->prevTime = currentTime;
  }
}
void trackPoseTask(void *ignore)
{
  while(true)
  {
    trackPosition(&globalPose, getLeftEncoder(), getRightEncoder(), getBackEncoder());
    trackVelocity(&globalVel, globalPose);
    delay(1);
  }
}

/*
double distanceToLineFromRobot(LineTarget *targetLine)
{
  return distanceToLine(targetLine, globalPose.x, globalPose.y);
}
double distanceToPointFromRobot(double targetX, double targetY)
{
  return distanceToPoint(globalPose.x, globalPose.y, targetX, targetY);
}
double angleToFacePointFromRobot(double targetX, double targetY)
{
  return nearestEquivalentAngleFromRobot(angleToFacePoint(globalPose.x, globalPose.y, targetX, targetY));
}
*/
double nearestEquivalentAngleFromRobot(double target)
{
  return nearestEquivalentAngle(globalPose.angle, target);
}
double calculateTravelDistance(int encoderCount, double wheelDiameter, int encoderResolution)
{
  return ((double) encoderCount * M_PI * wheelDiameter) / ((double) encoderResolution);
}
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
}
void resetRightEncoder()
{
  encoderReset(rightEncoder);
}
void resetBackEncoder()
{
  encoderReset(backEncoder);
}
