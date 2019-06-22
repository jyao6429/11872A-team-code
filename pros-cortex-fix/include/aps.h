#ifndef APS_H
#define APS_H

#include "main.h"

// Enum for Pose
enum Pose
{
  POSE_X,
  POSE_Y,
  POSE_ANGLE
};

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

// Previous encoder values
int prevLeftEncoder;
int prevRightEncoder;
int prevBackEncoder;
/**
 * The previous position vector and orientation calculated by the APS
 */
double robotPose[3];
/**
 * The angle at last reset
 */
double resetAngle;

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

/**
 * Initializes the Absolute Positioning System
 *
 * @param startX - the starting x coordinate in inches
 * @param startY - the starting y coordinate in inches
 * @param startAngle - the starting angle in degrees
 */
void initializeAPS(double startX, double startY, double startAngle);
/**
 * Resets the Absolute Positioning System
 *
 * @param resetX - the reset x coordinate in inches
 * @param resetY - the reset y coordinate in inches
 * @param resetAngle - the reset angle in degrees
 */
void resetPosition(double resetX, double resetY, double resetA);
/**
 * Starts tracking the robot position
 * This shoud be started as a task
 */
void startTracking(void *ignore);

double distanceToLineFromRobot(LineTarget *targetLine);
/**
 * Calculates the distance from the current robot position to a point
 *
 * @param targetX - the x component of the target point
 * @param targetY - the y component of the target point
 *
 * @return the absolute distance between the robot and target point
 */
double distanceToPointFromRobot(double targetX, double targetY);
/**
 * Calculate the orientation of the robot to face a certain point
 *
 * @param targetX - the x component of the target point
 * @param targetY - the y component of the target point
 *
 * @return the angle to face target point from the current robot position in the range [-PI, PI]
 */
double angleToFacePointFromRobot(double targetX, double targetY);
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

#endif
