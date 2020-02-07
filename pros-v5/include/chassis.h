#ifndef CHASSIS_H
#define CHASSIS_H

#include "main.h"

/**
 * Powers the motors on the chassis using voltage control
 *
 * @param leftPower - (-1.0 - 1.0) the power for the left wheels
 * @param rightPower - (-1.0 - 1.0) the power  for the right wheels
 */
void setDrive(double leftPower, double rightPower);
/**
 * Legacy function that powers the motors on the chassis using voltage control
 *
 * @param leftPower - (-127 - 127) the power for the left wheels
 * @param rightPower - (-127 - 127) the power  for the right wheels
 */
void setDrive(int leftPower, int rightPower);
/**
 * Powers the motors on the chassis using velocity control
 *
 * @param leftSpeed - the rpm for the left wheels
 * @param rightSpeed - the rpm  for the right wheels
 */
void setDriveVel(double leftSpeed, double rightSpeed);
/**
 * Stops all the drive motors
 */
void stopDrive();

#endif
