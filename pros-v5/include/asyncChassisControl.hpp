#ifndef ASYNC_CHASSIS_CONTROL_H
#define ASYNC_CHASSIS_CONTROL_H

#include "main.h"

// Enums for each different type of motion
typedef enum AsyncChassisOptions
{
  ASYNC_CHASSIS_NONE,
  ASYNC_MTT_SIMPLE,
  ASYNC_MTT_DIS,
  ASYNC_TTT_SWEEP,
  ASYNC_TTT_ANGLE,
  ASYNC_TTT_TARGET
} AsyncChassisOptions;

// Structs as containers for motion parameters
// Container for moveToTarget motions
typedef struct MTTContainer
{
  double angle, distance, targetX, targetY, startX, startY, maxErrorX, decelEarly, dropEarly;
  int power, startPower, decelPower;
  StopType stopType;
  MTTMode mode;
  bool isDegrees;
} MTTContainer;
// Container for sweepTurnToTarget motion
typedef struct TTTSweepContainer
{
  double targetX, targetY, targetAngle, targetRadius;
  TurnDir turnDir;
  int power;
  bool isAccurate, isDegrees;
} TTTSweepContainer;
// Container for turnToAngleNew and turnToTargetNew motions
typedef struct TTTRegularContainer
{
  double targetX, targetY, targetAngle, fullPowerRatio, stopPowerDiff, angleOffset;
  TurnDir turnDir;
  int coastPower;
  bool harshStop, isDegrees;
} TTTRegularContainer;

/**
 * Waits until the robot completes the current motion
 */
void waitUntilChassisMoveComplete();
/**
 * Queues the async controller task
 */
void queueAsyncChassisController(AsyncChassisOptions moveToQueue);
/**
 * Stops the async controller and resets variables to defaults
 */
void stopAsyncChassisController();

/**
 * Moves to a desired position along a line connecting the target and starting points
 *
 * @param targetX - the x component of the target point
 * @param targetY - the y component of the target point
 * @param startX - the x component of the starting point
 * @param startY - the y component of the starting point
 * @param power - (-127 - 127) the power to travel the line, negatives means the robot travels backwards
 * @param startPower - the starting power to ramp up to power (No use with MTT_SIMPLE)
 * @param maxErrorX - (e.g. 0.5 - 1) the maxiumum error perpendicular to the line in inches
 * @param decelEarly - (0 - 20) the distance from the target when to use the constant decelPower
 * @param decelPower - (0 - 50) the power to use when decelerating
 * @param dropEarly - (0 - 15) the distance from the target when to start breaking if needed, this also affects the deceleration point
 * @param stopType - the StopType to use to park
 * @param mode - the MTTMode to use to travel the line
 */
void moveToTargetSimpleAsync(double targetX, double targetY, double startX, double startY, int power, int startPower, double maxErrorX, double decelEarly, int decelPower, double dropEarly, StopType stopType, MTTMode mode);
/**
 * Moves a desired distance along a line in the direction of the given angle
 *
 * @param angle - the orientation to move in
 * @param distance - the distance to travel in inches
 * @param startX - the x component of the starting point
 * @param startY - the y component of the starting point
 * @param power - (-127 - 127) the power to travel the line, negatives means the robot travels backwards
 * @param startPower - the starting power to ramp up to power (No use with MTT_SIMPLE)
 * @param maxErrorX - (e.g. 0.5 - 1) the maxiumum error perpendicular to the line in inches
 * @param decelEarly - (0 - 20) the distance from the target when to use the constant decelPower
 * @param decelPower - (0 - 50) the power to use when decelerating
 * @param dropEarly - (0 - 15) the distance from the target when to start breaking if needed, this also affects the deceleration point
 * @param stopType - the StopType to use to park
 * @param mode - the MTTMode to use to travel the line
 * @param isDegrees - if the angle is given in degrees
 */
void moveToTargetDisSimpleAsync(double angle, double distance, double startX, double startY, int power, int startPower, double maxErrorX, double decelEarly, int decelPower, double dropEarly, StopType stopType, MTTMode mode, bool isDegrees);
/**
 * Turns to face a specific point with an offset if desired
 *
 * @param targetX - the x component of the target point
 * @param targetY - the y component of the target point
 * @param targetAngle - the angle to sweep to
 * @param targetRadius - (e.g. 10 - 20) the radius of the circle to follow in inches
 * @param turnDir - the direction to turn based on TurnDir enum (TURN_CH = chooses automatically)
 * @param power - (-127 - 127) the power to take the turn, negatives means the robot travels backwards
 * @param isAccurate - if the sweeps should be more accurate
 * @param isDegrees - if the targetAngle is given in degrees
 */
void sweepTurnToTargetAsync(double targetX, double targetY, double targetAngle, double targetRadius, TurnDir turnDir, int power, bool isAccurate, bool isDegrees);
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
void turnToAngleNewAsync(double targetAngle, TurnDir turnDir, double fullPowerRatio, int coastPower, double stopPowerDiff, bool harshStop, bool isDegrees);
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
 * @param isDegrees - if the angleOffset is given in degrees
 */
void turnToTargetNewAsync(double targetX, double targetY, TurnDir turnDir, double fullPowerRatio, int coastPower, double stopPowerDiff, double angleOffset, bool harshStop, bool isDegrees);

#endif
