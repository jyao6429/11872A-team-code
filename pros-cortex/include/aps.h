#ifndef APS_H
#define APS_H

// Enums for Cartesian and Polar coordinates
enum Cartesian
{
  X_COMP,
  Y_COMP
};
enum Polar
{
  MAGNITUDE,
  ANGLE
};
// Previous encoder values
int prevLeft;
int prevRight;
int prevBack;
/**
 * The previous position vector calculated by the APS
 */
double prevPos[2];
/**
 * The previous angle calculated by the APS
 */
double prevAngle;
/**
 * The angle at last reset
 */
double resetAngle;

/**
 * Gets the current count for the left tracking encoder
 *
 * @return the number of ticks
 */
int getLeftEncoder();
/**
 * Gets the current count for the right tracking encoder
 *
 * @return the number of ticks
 */
int getRightEncoder();
/**
 * Gets the current count for the back tracking encoder
 *
 * @return the number of ticks
 */
int getBackEncoder();
/**
 * Resets the left encoder
 */
void resetLeftEncoder();
/**
 * Resets the right encoder
 */
void resetRightEncoder();
/**
 * Resets the back encoder
 */
void resetBackEncoder();

/**
 * Initializes the Absolute Positioning System
 *
 * @param startX - the starting x coordinate in inches
 * @param startY - the starting y coordinate in inches
 * @param startAngle - the starting angle in degrees
 */
void initializeAPS(double startX, double startY, double startAngle);
/**
 * Resets the Absolute Positioning System
 *
 * @param resetX - the reset x coordinate in inches
 * @param resetY - the reset y coordinate in inches
 * @param resetAngle - the reset angle in degrees
 */
void resetPosition(double resetX, double resetY, double resetAngle);
/**
 * Starts integrating the gyro rate to keep track of orientation
 * This should be started as a task
 */
void startGyroIntegral(void *ignore);
/**
 * Starts tracking the robot position
 * This shoud be started as a task
 */
void startTracking(void *ignore);

/**
 * Calculates nearest equivalent angle in radians
 *
 * @param ref - the current orientation in radians
 * @param target - the target orientation in radians
 *
 * @return the target orientation + 2 x pi x k added
 */
double nearestEquivalentAngle(double ref, double target);
/**
 * Converts cartesian coordinates into polar
 *
 * @param *source - the cartesian array to convert from
 * @param *target - the polar array to convert to
 */
void convertPolar(double *source, double *target);
/**
 * Converts polar coordinates into cartesian
 *
 * @param *source - the polar array to convert from
 * @param *target - the cartesian array to convert to
 */
void convertCart(double *source, double *target);
/**
 * Converts encoder counts into linear distance traveled in the units of the diameter of the tracking wheel
 *
 * @param encoderCount - the number of encoder ticks
 * @param wheelDiameter - the diameter of the tracking wheel
 * @param encoderResolution - the number of encoder ticks per 360 degrees
 *
 * @return the distance traveled by that wheel
 */
double calculateTravelDistance(int encoderCount, double wheelDiameter, int encoderResolution);
/**
 * Converts degrees to radians
 *
 * @param degrees - the angle to convert in degrees
 * @return the converted angle in radians
 */
double degToRad(double degrees);
/**
 * Converts radians to degrees
 *
 * @param rads - the angle to convert in radians
 * @return the converted angle in degrees
 */
double radToDeg(double rads);

#endif
