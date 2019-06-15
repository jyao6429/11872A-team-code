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
 * Drives in a straight line to a point
 *
 * @param targetX - the x component of the target point
 * @param targetY - the y component of the target point
 * @param maxSpeed - the maximum speed to take
 * @param isAccurate - if the drive needs to be accurate
 */
void driveStraightToPoint(double targetX, double targetY, int maxSpeed, bool isAccurate);
/**
 * Turns to a specific orientation
 *
 * @param targetAngle - the target orientation in degrees
 * @param maxSpeed - the maximum speed to take the turn
 * @param isAccurate - if the turn needs to be accurate
 */
void turnToAngle(double targetAngle, int maxSpeed, bool isAccurate);
#endif
