#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include "aps.h"
#include "main.h"

// Structs
typedef struct Cart
{
  double x, y;
} Cart;

typedef struct Polar
{
  double magnitude, angle;
} Polar;

typedef struct Line
{
  Pose p1, p2;
} Line;

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

double getAngleOfLine(Line line);
double getLengthOfLine(Line line);
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
  * @param cartVector - the cartesian vector to convert from
  * @param *polarVector - the polar vector to convert to
  */
void cartToPolar(Cart cartVector, Polar *polarVector);
 /**
  * Converts polar coordinates into cartesian
  *
  * @param polarVector - the polar vector to convert from
  * @param *cartVector - the cartesian vector to convert to
  */
void polarToCart(Polar polarVector, Cart *cartVector);


#endif
