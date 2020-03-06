#ifndef ASYNC_TRAY_CONTROL_H
#define ASYNC_TRAY_CONTROL_H

#include "main.h"

#define TRAY_ANGLED 310
#define TRAY_VERTICAL 2500
#define TRAY_ARM 600

// Variables for handling async tasks
// Keeps track if the tray is at the target
extern bool isTrayAtTarget;
// The next target pot value for the tray
extern int nextTrayTarget;

/**
 * Initialized the tray motor and controller
 */
void initTray();
bool waitUntilTrayMoveComplete(int timeout);
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
 * Pauses the async controller and resets variables to defaults
 */
void pauseAsyncTrayController();
/**
 * Moves the tray to a designated position
 *
 * @param trayTarget - the target position for the tray
 */
void moveTrayToPosition(int trayTarget);
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
 * @param power - (-1.0 - 1.0) the power to set the motor to
 */
void setTray(double power);
/**
 * Sets the motor power for the tray
 *
 * @param power - (-127 - 127) the power to set the motor to
 */
void setTray(int power);
/**
 * Sets the motor velocity for the tray
 *
 * @param speed - (-100 - 100) the velocity to set the motor to
 */
void setTrayVel(double speed);
/**
 * Stops the tray motor
 */
void stopTray();
bool isTrayMotorOverTemp();
#endif
