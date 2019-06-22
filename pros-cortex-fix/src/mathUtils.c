#include "main.h"

void matchLineWithPose(LineTarget *targetLine, double targetX, double targetY, double targetAngle, bool isDegrees)
{
  if (isDegrees)
    targetAngle = degToRad(targetAngle);

  double slope = tan(targetAngle + M_PI);

  targetLine->a = -slope;
  targetLine->b = 1;
  targetLine->c = slope * targetX - targetY;
  targetLine->targetX = targetX;
  targetLine->targetY = targetY;
  targetLine->targetAngle = targetAngle;
}
double distanceToPoint(double sourceX, double sourceY, double targetX, double targetY)
{
  return sqrt(pow(targetX - sourceX, 2) + pow(targetY - sourceY, 2));
}
double angleToFacePoint(double sourceX, double sourceY, double targetX, double targetY)
{
  return -atan2(targetY - sourceY, targetX - sourceX) + M_PI / 2;
}
double nearestEquivalentAngle(double source, double target)
{
  return round((source - target) / (2 * M_PI)) * 2 * M_PI + target;
}
double normalizeAngle(double angle)
{
  return atan2(sin(angle), cos(angle));
}
double degToRad(double degrees)
{
  return degrees * (M_PI / 180);
}
double radToDeg(double radians)
{
  return radians * (180 / M_PI);
}
void cartToPolar(double *cartVector, double *polarVector)
{
  // Calculates magnitude of vector with distance formula
  polarVector[MAGNITUDE] = sqrt(pow(cartVector[X_COMP], 2) + pow(cartVector[Y_COMP], 2));
  // Calculates angle with arctan, automatically gives angle in correct quadrant
  polarVector[ANGLE] = atan2(cartVector[Y_COMP], cartVector[X_COMP]);
}
void polarToCart(double *polarVector, double *cartVector)
{
  // Calculate x component with cosine
  cartVector[X_COMP] = polarVector[MAGNITUDE] * cos(polarVector[ANGLE]);
  // Calculate y component with sine
  cartVector[Y_COMP] = polarVector[MAGNITUDE] * sin(polarVector[ANGLE]);
}
