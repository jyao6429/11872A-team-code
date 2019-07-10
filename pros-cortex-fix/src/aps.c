#include "main.h"

void resetPositionFull(Pose *position, double startX, double startY, double startAngle, bool isDegrees)
{
  // Convert to radians if needed
  if (isDegrees)
    startAngle = degToRad(startAngle);

  printf("(resetPositionFull) X: %3.3f   Y: %3.3f   A: %3.3f\n", startX, startY, radToDeg(startAngle));

  // Stop task
  unsigned int APSState = taskGetState(APSTask);
  if (APSTask != NULL && (APSState == TASK_RUNNING || APSState == TASK_SLEEPING || APSState == TASK_SUSPENDED))
    taskDelete(APSTask);

//print("Deleted task if needed\n");

  // Reset everything
  resetPosition(position);

  // Reset all the encoders
  resetLeftEncoder();
  resetRightEncoder();
  resetBackEncoder();

//print("Resetted everyting\n");

  // Set the new positions
  resetAngle = startAngle;
  position->angle = startAngle;
  position->x = startX;
  position->y = startY;

//print("Set new data\n");

  // Create new task to track position
  APSTask = taskCreate(trackPoseTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT + 1);

//print("Task created\n");
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
  double deltaLeftDistance = calculateTravelDistance(currentLeft - position->prevLeft, SIDE_WHEEL_DIAMETER, SIDE_ENCODER_RESOLUTION);
  double deltaRightDistance = calculateTravelDistance(currentRight - position->prevRight, SIDE_WHEEL_DIAMETER, SIDE_ENCODER_RESOLUTION);
  double deltaBackDistance = calculateTravelDistance(currentBack - position->prevBack, BACK_WHEEL_DIAMETER, BACK_ENCODER_RESOLUTION);

  // Update prev values;
  position->prevLeft = currentLeft;
  position->prevRight = currentRight;
  position->prevBack = currentBack;

  // Calculate change in angle
  double deltaAngle = (deltaLeftDistance - deltaRightDistance) / (SL + SR);

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
    localOffset.x = 2.0 * sin(deltaAngle / 2.0) * ((deltaBackDistance / deltaAngle) + SB);
    localOffset.y = 2.0 * sin(deltaAngle / 2.0) * ((deltaRightDistance / deltaAngle) + SR);
  }

  // Calculate average angle
  double avgAngle = position->angle + (deltaAngle / 2);

  // Calculate cosine and sine of avgAngle
  double cosAvg = cos(avgAngle);
  double sinAvg = sin(avgAngle);

  // Update the global position by incrementing by a shifted localOffset
  position->x += localOffset.x * cosAvg + localOffset.y * sinAvg;
  position->y += localOffset.x * -sinAvg + localOffset.y * cosAvg;
  position->angle += deltaAngle;
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
bool isRobotStopped()
{
  return (fabs(globalVel.x) < 0.5) && (fabs(globalVel.y) < 0.5) && (fabs(globalVel.angle) < M_PI / 30);
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
