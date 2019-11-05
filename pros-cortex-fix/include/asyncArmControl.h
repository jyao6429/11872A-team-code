#ifndef ASYNC_ARM_CONTROL_H
#define ASYNC_ARM_CONTROL_H

#include "main.h"

// Variables for handling async tasks
// Keeps track when the Arm is moving
bool isArmAtTarget;
// Handles the async task
TaskHandle asyncArmHandle;
// The next move to be performed by the robot
int nextArmTarget;

/**
 * Waits until the arm completes the current motion
 */
void waitUntilArmMoveComplete();
/**
 * Starts the async controller and resets variables to defaults
 */
void startAsyncArmController();
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

#endif
