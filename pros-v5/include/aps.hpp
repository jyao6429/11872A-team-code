#ifndef APS_H
#define APS_H

#include "main.h"

// Physical parameters in inches
// distance from center to left tracking wheel
#define SL 4.43436
// distance from center to right tracking wheel
#define SR 4.43436
// distance from center to back tracking wheel
#define SB 5.36208
// diameter of side wheels
#define SIDE_WHEEL_DIAMETER 2.7382
// diameter of back wheel
#define BACK_WHEEL_DIAMETER 2.7382
// side encoder ticks per 360 degrees of motion
#define SIDE_ENCODER_RESOLUTION 360
// back encoder ticks per 360 degrees of motion
#define BACK_ENCODER_RESOLUTION 360
// distance from back of the robot to center
#define BACK_TO_CENTER 8.75
// distance between opposite field walls
#define FIELD_WIDTH 140.5

// Structs for pose and velocity
// Container for position and orientation
typedef struct Pose
{
  double angle, x, y;
  int prevLeft, prevRight, prevBack;
} Pose;
// Container for linear and angular velocities
typedef struct Vel
{
  double angle, x, y, prevPoseAngle, prevPoseX, prevPoseY;
  unsigned long prevTime;
} Vel;

// Variables for global position and velocity
// The global position of the robot
extern Pose globalPose;
// The global velocity of the robot
extern Vel globalVel;
// The angle at last reset
extern double resetAngle;

/**
 * Stops position and velocity tracking
 */
void stopAPS();
/**
 * Resets the position to given parameters, and restarts the APS
 *
 * @param *position - the position struct
 * @param startX - the starting x coordinate in inches
 * @param startY - the starting y coordinate in inches
 * @param startAngle - the starting angle in degrees
 * @param isDegrees - if startAngle is given in degrees
 */
void resetPositionFull(Pose *position, double startX, double startY, double startAngle, bool isDegrees);
/**
 * Resets the given pose to 0
 *
 * @param *position - the position struct
 */
void resetPosition(Pose *position);
/**
 * Resets the given velocity to 0, with prevPoses as the given position struct
 *
 * @param *velocity - the velocity struct
 * @param position - the position struct
 */
void resetVelocity(Vel *velocity, Pose position);
/**
 * Starts tracking the robot position and velocity
 * This shoud be started as a task
 */
void trackPoseTask(void *ignore);
/**
 * Checks if the robot is moving or not
 *
 * @return if the robot is stopped or not, allowing some small movements
 */
bool isRobotStopped();
/**
 * Calculates nearest equivalent angle in radians from current orientation
 *
 * @param target - the target orientation in radians
 *
 * @return the target orientation + 2 x pi x k added
 */
double nearestEquivalentAngleFromRobot(double target);
/**
 * Converts encoder counts into linear distance traveled in the units of the diameter of the tracking wheel
 *
 * @param encoderCount - the number of encoder ticks
 * @param wheelDiameter - the diameter of the tracking wheel
 * @param encoderResolution - the number of encoder ticks per 360 degrees
 *
 * @return the distance traveled by that wheel
 */
double calculateTravelDistance(int encoderCount, double wheelDiameter, int encoderResolution);
/**
 * Gets the current count for the left tracking encoder
 *
 * @return the number of ticks
 */
int getLeftEncoder();
/**
 * Gets the current count for the right tracking encoder
 *
 * @return the number of ticks
 */
int getRightEncoder();
/**
 * Gets the current count for the back tracking encoder
 *
 * @return the number of ticks
 */
int getBackEncoder();
/**
 * Resets the left encoder
 */
void resetLeftEncoder();
/**
 * Resets the right encoder
 */
void resetRightEncoder();
/**
 * Resets the back encoder
 */
void resetBackEncoder();

#endif
