#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include "main.h"

// Enums for Cartesian and Polar
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

// Struct for line travelling information
typedef struct
{
  double a, b, c, targetX, targetY, targetAngle;
} LineTarget;

double distanceToLine(LineTarget *targetLine, double sourceX, double sourceY);
void matchLineWithPose(LineTarget *targetLine, double targetX, double targetY, double targetAngle, bool isDegrees);
/**
 * Calculates the distance from the current position to a point
 *
 * @param sourceX - the x component of the source point
 * @param sourceY - the y component of the source point
 * @param targetX - the x component of the target point
 * @param targetY - the y component of the target point
 *
 * @return the absolute distance between the two points
 */
double distanceToPoint(double sourceX, double sourceY, double targetX, double targetY);
/**
 * Calculate the orientation of the robot to face a certain point
 *
 * @param sourceX - the x component of the source point
 * @param sourceY - the y component of the source point
 * @param targetX - the x component of the target point
 * @param targetY - the y component of the target point
 *
 * @return the angle to face target point from source point in the range [-PI, PI]
 */
double angleToFacePoint(double sourceX, double sourceY, double targetX, double targetY);
/**
 * Calculates nearest equivalent angle in radians
 *
 * @param target - the target orientation in radians
 *
 * @return the target orientation + 2 x pi x k added
 */
 double nearestEquivalentAngle(double source, double target);
/**
 * Gives back the equivalent angle in the range -PI to PI
 *
 * @param angle - the angle to normalize
 *
 * @return the angle in the range -PI to PI
 */
double normalizeAngle(double angle);
/**
 * Converts degrees to radians
 *
 * @param degrees - the angle to convert in degrees
 *
 * @return the converted angle in radians
 */
double degToRad(double degrees);
/**
 * Converts radians to degrees
 *
 * @param radians - the angle to convert in radians
 *
 * @return the converted angle in degrees
 */
 double radToDeg(double radians);
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


#endif
