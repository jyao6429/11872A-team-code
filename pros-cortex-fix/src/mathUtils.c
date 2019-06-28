#include "main.h"

double distanceToLine(LineTarget *targetLine, double sourceX, double sourceY)
{
  return (targetLine->a * sourceX + targetLine->b * sourceY + targetLine->c) / sqrt(pow(targetLine->a, 2) + pow(targetLine->b, 2));
}
void matchLineWithPose(LineTarget *targetLine, double targetX, double targetY, double targetAngle, bool isDegrees)
{
  if (isDegrees)
    targetAngle = degToRad(targetAngle);

  double slope = tan(-targetAngle + M_PI / 2);

  if (fabs(slope) > 1000000)
  {
    targetLine->a = -1;
    targetLine->b = 0;
    targetLine->c = targetX;
  }
  else
  {
    targetLine->a = -slope;
    targetLine->b = 1;
    targetLine->c = slope * targetX - targetY;
  }
  targetLine->targetX = targetX;
  targetLine->targetY = targetY;
  targetLine->targetAngle = targetAngle;

  // Debug
  printf("a: %3.3f   b: %3.3f   c: %3.3f   TX: %3.3f   TY: %3.3f   TA: %3.3f\n", targetLine->a, targetLine->b, targetLine->c, targetX, targetY, targetAngle);
}
double distanceToPoint(double sourceX, double sourceY, double targetX, double targetY)
{
  return sqrt(pow(targetX - sourceX, 2) + pow(targetY - sourceY, 2));
}
double angleToFacePoint(double sourceX, double sourceY, double targetX, double targetY)
{
  return -atan2(targetY - sourceY, targetX - sourceX) + M_PI / 2;
}

double getAngleOfLine(Line line)
{
  // Note, since angle is calculated from y axis, x and y are flipped
  return atan2(line.p2.x - line.p1.x, line.p2.y - line.p1.y);
}
double getLengthOfLine(Line line)
{
  return sqrt(pow(line.p2.x - line.p1.x, 2) + pow(line.p2.y - line.p1.y, 2));
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
void cartToPolar(Cart cartVector, Polar *polarVector)
{
  if (cartVector.x != 0.0 || cartVector.y != 0.0)
  {
    // Calculates magnitude of vector with distance formula
    polarVector->magnitude = sqrt(pow(cartVector.x, 2) + pow(cartVector.y, 2));
    // Calculates angle with arctan, automatically gives angle in correct quadrant
    polarVector->angle = atan2(cartVector.y, cartVector.x);
  }
  else
  {
    polarVector->magnitude = polarVector->angle = 0.0;
  }
}
void polarToCart(Polar polarVector, Cart *cartVector)
{
  // Calculate x component with cosine
  cartVector->x = polarVector.magnitude * cos(polarVector.angle);
  // Calculate y component with sine
  cartVector->y = polarVector.magnitude * sin(polarVector.angle);
}
