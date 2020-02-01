#ifndef ASYNC_TRAY_CONTROL_H
#define ASYNC_TRAY_CONTROL_H

#include "main.h"

#define TRAY_ANGLED 20
#define TRAY_VERTICAL 2030
#define TRAY_ARM 900

// Variables for handling async tasks
// Keeps track if the tray is at the target
bool isTrayAtTarget;
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
/**
 * Gets the value from the tray potentiometer
 *
 * @returns the value from the tray pot
 */
int getTrayPot();
/**
 * Sets the motor power for the tray
 *
 * @param power - the power to set the motor to
 */
void setTray(int power);
/**
 * Stops the tray motor
 */
void stopTray();
#endif
