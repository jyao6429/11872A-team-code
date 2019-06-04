#ifndef APS_H
#define APS_H

// Enums for Pose, Cartesian and Polar
enum Pose
{
  POSE_X,
  POSE_Y,
  POSE_ANGLE
};
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
int prevLeftEncoder;
int prevRightEncoder;
int prevBackEncoder;
/**
 * Angle calculated by gyro
 */
double gyroAngle;
/**
 * The previous position vector and orientation calculated by the APS
 */
double robotPose[3];
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
 * @param reference - the current orientation in radians
 * @param target - the target orientation in radians
 *
 * @return the target orientation + 2 x pi x k added
 */
double nearestEquivalentAngle(double reference, double target);
/**
 * Converts cartesian coordinates into polar
 *
 * @param *cartVector - the cartesian array to convert from
 * @param *polarVector - the polar array to convert to
 */
void cartToPolar(double *cartVector, double *polarVector);
/**
 * Converts polar coordinates into cartesian
 *
 * @param *polarVector - the polar array to convert from
 * @param *cartVector - the cartesian array to convert to
 */
void polarToCart(double *polarVector, double *cartVector);
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
 * @param radians - the angle to convert in radians
 * @return the converted angle in degrees
 */
 double radToDeg(double radians);

#endif
