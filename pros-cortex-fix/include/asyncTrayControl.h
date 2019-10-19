#ifndef ASYNC_TRAY_CONTROL_H
#define ASYNC_TRAY_CONTROL_H

#include "main.h"

// Enums for each different type of motion
typedef enum AsyncTrayOptions
{
  ASYNC_TRAY_NONE,
  ASYNC_TRAY_VERTICAL,
  ASYNC_TRAY_ANGLED
} AsyncTrayOptions;

// Variables for handling async tasks
// Keeps track when the tray is moving
bool isTrayMoving;
// Handles the async task
TaskHandle asyncTrayHandle;
// The next move to be performed by the robot
AsyncTrayOptions nextTrayMove;

/**
 * Waits until the tray completes the current motion
 */
void waitUntilTrayMoveComplete();
/**
 * Queues the async controller task
 */
void queueAsyncTrayController(AsyncTrayOptions moveToQueue);
/**
 * Stops the async controller and resets variables to defaults
 */
void stopAsyncTrayController();
/**
 * Moves the tray to the vertical position for scoring
 */
void moveTrayVerticalAsync();
/**
 * Moves the tray to the angled position for cube intake
 */
void moveTrayAngledAsync();

#endif
