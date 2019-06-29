#ifndef MOTION_ALGORITHMS_H
#define MOTION_ALGORITHMS_H

#include "main.h"

// Enums
typedef enum TurnDir
{
  TURN_CW,
  TURN_CCW,
  TURN_CH
} TurnDir;

typedef enum StopType
{
  STOP_NONE,
  STOP_SOFT,
  STOP_HARSH
} StopType;

typedef enum MTTMode
{
  MTT_SIMPLE,
  MTT_PROPORTIONAL,
  MTT_CASCADING
} MTTMode;

// Coordinates of last target
Cart lastTarget;

/**
 * Turns to a specified orientation
 *
 * @param targetAngle - the angle to turn to
 * @param turnDir - the direction to turn based on TurnDir enum (TURN_CH = chooses automatically)
 * @param fullPowerRatio - (0.0 - 1.0) ratio of the turn taken in full power, depends on speed and accuracy needed
 * @param coastPower - (e.g. 20 - 30) the power used to turn after full power, also depends on speed and accuracy needed
 * @param stopPowerDiff - (e.g. 10 - 15) the angle error in degrees to cut the motors
 * @param harshStop - if the robot should brake at the end of the turn
 * @param isDegrees - if the targetAngle is given in degrees
 */
void turnToAngleNew(double targetAngle, TurnDir turnDir, double fullPowerRatio, int coastPower, double stopPowerDiff, bool harshStop, bool isDegrees);
/**
 * Turns to face a specific point with an offset if desired
 *
 * @param targetX - the x component of the target point
 * @param targetY - the y component of the target point
 * @param turnDir - the direction to turn based on TurnDir enum (TURN_CH = chooses automatically)
 * @param fullPowerRatio - (0.0 - 1.0) ratio of the turn taken in full power, depends on speed and accuracy needed
 * @param coastPower - (e.g. 20 - 30) the power used to turn after full power, also depends on speed and accuracy needed
 * @param stopPowerDiff - (e.g. 10 - 15) the angle error in degrees to cut the motors
 * @param angleOffset - the desired offset to the angle directly facing the point
 * @param harshStop - if the robot should brake at the end of the turn
 */
void turnToTargetNew(double targetX, double targetY, TurnDir turnDir, double fullPowerRatio, int coastPower, double stopPowerDiff, double angleOffset, bool harshStop);
/**
 * Harshly stops the robot
 */
void applyHarshStop();

#endif
