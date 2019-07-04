#ifndef APS_H
#define APS_H

#include "main.h"

// Physical parameters in inches
// distance from center to left tracking wheel
const double sL;
// distance from center to right tracking wheel
const double sR;
// distance from center to back tracking wheel
const double sB;
// diameter of side wheels
const double sideWheelDiameter;
// diameter of back wheel
const double backWheelDiameter;
// side encoder ticks per 360 degrees of motion
const int sideEncoderResolution;
// back encoder ticks per 360 degrees of motion
const int backEncoderResolution;

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
Pose globalPose;
// The global velocity of the robot
Vel globalVel;
// The angle at last reset
double resetAngle;

// handles the tracking task
TaskHandle APSTask;

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

//double distanceToLineFromRobot(LineTarget *targetLine);
/**
 * Calculates the distance from the current robot position to a point
 *
 * @param targetX - the x component of the target point
 * @param targetY - the y component of the target point
 *
 * @return the absolute distance between the robot and target point
 */
//double distanceToPointFromRobot(double targetX, double targetY);
/**
 * Calculate the orientation of the robot to face a certain point
 *
 * @param targetX - the x component of the target point
 * @param targetY - the y component of the target point
 *
 * @return the angle to face target point from the current robot position in the range [-PI, PI]
 */
//double angleToFacePointFromRobot(double targetX, double targetY);
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
