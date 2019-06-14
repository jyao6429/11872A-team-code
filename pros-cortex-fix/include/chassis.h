#ifndef CHASSIS_H
#define CHASSIS_H

/**
 * Powers the motors on the chassis
 *
 * @param leftPower - the power for the left wheels
 * @param rightPower - the power  for the right wheels
 */
void powerMotors(int leftPower, int rightPower);
/**
 * Turns to a specifig orientation
 * @param targetAngle - the target orientation in degrees
 * @param maxSpeed - the maximum speed to take the turn
 * @param isAccurate - if the turn needs to be accurate
 */
void turnToAngle(double targetAngle, int maxSpeed, bool isAccurate);
#endif
