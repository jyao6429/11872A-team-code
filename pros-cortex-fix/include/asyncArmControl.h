#ifndef ASYNC_ARM_CONTROL_H
#define ASYNC_ARM_CONTROL_H

#include "main.h"

// Enums for each different type of motion
typedef enum AsyncArmOptions
{
  ASYNC_ARM_NONE,
  ASYNC_ARM_ZERO,
  ASYNC_ARM_SCORE,
  ASYNC_ARM_LOW,
  ASYNC_ARM_MED,
  ASYNC_ARM_PUSH_DOWN
} AsyncArmOptions;

// Variables for handling async tasks
// Keeps track when the Arm is moving
bool isArmMoving;
// Handles the async task
TaskHandle asyncArmHandle;
// The next move to be performed by the robot
AsyncArmOptions nextArmMove;

/**
 * Waits until the arm completes the current motion
 */
void waitUntilArmMoveComplete();
/**
 * Queues the async controller task
 */
void queueAsyncArmController(AsyncArmOptions moveToQueue);
/**
 * Stops the async controller and resets variables to defaults
 */
void stopAsyncArmController();
/**
 * Moves intake arms to the down position for cube intake
 */
void moveArmsZeroAsync();
/**
 * Holds intake arms to clear the intake lock in order to score in the goal zones
 */
void moveArmsScoreAsync();
/**
 * Holds intake arms to score/descore low towers
 */
void moveArmsLowAsync();
/**
 * Holds intake arms to score/descore medium towers
 */
void moveArmsMedAsync();
void pushArmsDownLoop();

#endif
