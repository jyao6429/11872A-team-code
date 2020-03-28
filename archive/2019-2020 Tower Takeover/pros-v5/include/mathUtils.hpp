#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include "aps.hpp"
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

/**
 * Modifies given coordinates to the nearest point to those coordinates on a given line
 *
 * @param *x - the pointer to the x coordinate
 * @param *y - the pointer to the y coordiante
 * @param m - the slope of the line
 * @param b - the y intercept of the line
 */
void nearestPointOnLine(double *x, double *y, double m, double b);
/**
 * Calculate the angle of the line with respect to the y axis
 *
 * @param line - the Line struct to calculate the angle to
 *
 * @return the angle of the line in the range [-PI, PI]
 */
double getAngleOfLine(Line line);
/**
 * Calculate the length of the line defined by two points
 *
 * @param line - the Line struct to calculate the distance to
 *
 * @return the disntance in inches of the line
 */
double getLengthOfLine(Line line);
/**
 * Calculates nearest equivalent angle in radians
 *
 * @param target - the target orientation in radians
 *
 * @return the target orientation + 2 x pi x k added
 */
double nearestEquivalentAngle(double angle, double reference);
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
 * Converts inches to centimeters
 *
 * @param inches - the length to convert in inches
 *
 * @return the converted length in centimeters
 */
double inToCM(double inches);
/**
 * Converts centimeters to inches
 *
 * @param centimeters - the length to convert in centimeters
 *
 * @return the converted length in inches
 */
double cmToIN(double centimeters);
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
