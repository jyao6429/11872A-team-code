#ifndef CHASSIS_H
#define CHASSIS_H

#include "main.h"

/**
 * Powers the motors on the chassis
 *
 * @param leftPower - the power for the left wheels
 * @param rightPower - the power  for the right wheels
 */
void powerMotors(int leftPower, int rightPower);
/**
 * Powers the motors on the chassis using TrueSpeed
 *
 * @param leftPower - the power for the left wheels
 * @param rightPower - the power  for the right wheels
 */
void powerMotorsLinear(int leftPower, int rightPower);
/**
 * Stops all the drive motors
 */
void stopMotors();

/**
 * Turns to a specific orientation
 *
 * @param targetAngle - the target orientation
 * @param maxSpeed - the maximum speed to take the turn
 * @param isAccurate - if the turn needs to be accurate
 * @param isDegrees - if targetAngle is given in degrees
 */
void turnToAngle(double targetAngle, int maxSpeed, bool isAccurate, bool isDegrees);
#endif
