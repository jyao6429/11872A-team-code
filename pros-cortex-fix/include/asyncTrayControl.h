#ifndef ASYNC_TRAY_CONTROL_H
#define ASYNC_TRAY_CONTROL_H

#include "main.h"

// Variables for handling async tasks
// Keeps track if the tray is at the target
bool isTrayAtTarget;
// Handles the async task
TaskHandle asyncTrayHandle;
// The next target pot value for the tray
int nextTrayTarget;

/**
 * Waits until the tray completes the current motion
 */
void waitUntilTrayMoveComplete();
/**
 * Starts the async controller and resets variables to defaults
 */
void startAsyncTrayController();
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
