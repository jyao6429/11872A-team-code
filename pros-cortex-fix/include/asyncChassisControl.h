#ifndef ASYNC_CHASSIS_CONTROL_H
#define ASYNC_CHASSIS_CONTROL_H

#include "main.h"

// Enums for each different type of motion
typedef enum AsyncChassisOptions
{
  ASYNC_NONE,
  ASYNC_MTT_SIMPLE,
  ASYNC_MTT_DIS,
  ASYNC_TTT_SWEEP,
  ASYNC_TTT_ANGLE,
  ASYNC_TTT_TARGET
} AsyncChassisOptions;

// Structs as containers for motion parameters
typedef struct MTTContainer
{
  double angle, distance, targetX, targetY, startX, startY, maxErrorX, decelEarly, dropEarly;
  int power, startPower, decelPower;
  StopType stopType;
  MTTMode mode;
  bool isDegrees;
} MTTContainer;

typedef struct TTTSweepContainer
{
  double targetX, targetY, targetAngle, targetRadius;
  TurnDir turnDir;
  int power;
  bool isAccurate, isDegrees;
} TTTSweepContainer;

typedef struct TTTRegularContainer
{
  double targetX, targetY, targetAngle, fullPowerRatio, stopPowerDiff, angleOffset;
  TurnDir turnDir;
  int coastPower;
  bool harshStop, isDegrees;
} TTTRegularContainer;

// Variables for handling async tasks
bool isChassisMoving;
TaskHandle asyncChassisHandle;
AsyncChassisOptions nextMove;
MTTContainer mttContainer;
TTTSweepContainer sweepContainer;
TTTRegularContainer turnContainer;

void initializeAsyncChassisController();
void stopAsyncChassisController();

void moveToTargetSimpleAsync(double targetX, double targetY, double startX, double startY, int power, int startPower, double maxErrorX, double decelEarly, int decelPower, double dropEarly, StopType stopType, MTTMode mode);
void moveToTargetDisSimpleAsync(double angle, double distance, double startX, double startY, int power, int startPower, double maxErrorX, double decelEarly, int decelPower, double dropEarly, StopType stopType, MTTMode mode, bool isDegrees);
void sweepTurnToTargetAsync(double targetX, double targetY, double targetAngle, double targetRadius, TurnDir turnDir, int power, bool isAccurate, bool isDegrees);
void turnToAngleNewAsync(double targetAngle, TurnDir turnDir, double fullPowerRatio, int coastPower, double stopPowerDiff, bool harshStop, bool isDegrees);
void turnToTargetNewAsync(double targetX, double targetY, TurnDir turnDir, double fullPowerRatio, int coastPower, double stopPowerDiff, double angleOffset, bool harshStop, bool isDegrees);

#endif
