#include "main.h"

void nearestPointOnLine(double *x, double *y, double m, double b)
{
  *x = (*y + *x / m - b) / (m + 1 / m);
  *y = m * *x + b;
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
double nearestEquivalentAngle(double angle, double reference)
{
  return round((reference - angle) / (2 * M_PI)) * 2 * M_PI + angle;
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
double inToCM(double inches)
{
  return inches * 2.54;
}
double cmToIN(double centimeters)
{
  return centimeters / 2.54;
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
  if (polarVector.magnitude != 0.0)
  {
    // Calculate x component with cosine
    cartVector->x = polarVector.magnitude * cos(polarVector.angle);
    // Calculate y component with sine
    cartVector->y = polarVector.magnitude * sin(polarVector.angle);
  }
  else
  {
    cartVector->x = cartVector->y = 0.0;
  }
}
