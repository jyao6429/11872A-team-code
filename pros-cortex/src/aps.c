#include "main.h"
#include "math.h"
#include "aps.h"
#include "constants.h"
#include "chassis.h"

// Physical parameters in inches
const double sL = 5.0;               // distance from center to left tracking wheel
const double sR = 5.0;               // distance from center to right tracking wheel
const double sB = 5.0;               // distance from center to back tracking wheel
const double wheelRadius = 2.0;      // radius of wheels
const int ticksPerRotation = 360;    // encoder ticks per 360 degrees of motion

// Previous positions
double prevPos[] = {0.0, 0.0};
double prevAngle = 0.0;
double resetAngle = 0.0;
int prevLeft = 0;
int prevRight = 0;
int prevBack = 0;

// Not sure if using v5 or cortex, just rewrite these functions to make code work
int getLeftEncoder() {return 0;}
int getRightEncoder() {return 0;}
int getBackEncoder() {return 0;}

void startTracking()
{
  while (true)
  {
    // Get current encoder values
    int currentLeft = getLeftEncoder();
    int currentRight = getRightEncoder();
    int currentBack = getBackEncoder();

    // Calculate traveled distance in inches
    double deltaLeft = wheelRadius * encoderToRad(currentLeft - prevLeft);
    double deltaRight = wheelRadius * encoderToRad(currentRight - prevRight);
    double deltaBack = wheelRadius * encoderToRad(currentBack - prevBack);

    // Update prev values;
    int prevLeft = currentLeft;
    int prevRight = currentRight;
    int prevBack = currentBack;

    // Calculate total change since last reset
    double totalLeft = wheelRadius * encoderToRad(currentLeft);
    double totalRight = wheelRadius * encoderToRad(currentRight);

    // Calculate new absolute orientation
    double newAngle = resetAngle + (totalLeft - totalRight) / (sL + sR);

    // Calculate change in angle
    double deltaAngle = newAngle - prevAngle;

    // Calculate local offset
    double localOffset[] = {0.0, 0.0};
    // If drove straight (about < 2 deg diff)
    if (fabs(deltaAngle) < 0.03)
    {
      localOffset[0] = deltaBack;
      localOffset[1] = deltaRight;
    }
    else
    {
      localOffset[0] = 2 * sin(deltaAngle / 2) * ((deltaBack / deltaAngle) + sB);
      localOffset[1] = 2 * sin(deltaAngle / 2) * ((deltaRight / deltaAngle) + sR);
    }

    // Calculate average angle
    double avgAngle = prevAngle + (deltaAngle / 2);

    // Calculate global offset
    double localPolar[] = {0.0, 0.0};

    // Convert localOffset to polar
    convertPolar(localOffset, localPolar);

    // Shift angle
    localPolar[1] += -avgAngle;
    double globalOffset[] = {0.0, 0.0};

    // Converting back to cartesian gives the globalOffset
    convertCart(localPolar, globalOffset);

    // Calculate new absolute position
    double currentPos[] = {prevPos[0] + globalOffset[0], prevPos[1] + globalOffset[1]};
    prevPos[0] = currentPos[0];
    prevPos[1] = currentPos[1];

    // Update previous angle

/* No need for this since we have nearestEquivalentAngle
    if (newAngle > 2 * M_PI)
      newAngle -= 2 * M_PI;
    if (newAngle < 0)
      newAngle += 2 * M_PI;
*/
    prevAngle = newAngle;

    delay(5);
  }
}

/**
* Calculates nearest equivalent angle in radians
*
* @param ref the current orientation in radians
* @param target the target orientation in radians
* @return the target orientation + 2 * pi * k added
*/
double nearestEquivalentAngle(double ref, double target)
{
  return round((ref - target) / (2 * M_PI)) * 2 * M_PI + target;
}
/**
* Converts cartesian coordinates into polar
*/
void convertPolar(double *source, double *target)
{
  double polar[2] = {0.0, 0.0};
  target[0] = sqrt(pow(source[0], 2) + pow(source[1], 2));
  double tempAngle = atan(source[1] / source[0]);

  if (source[0] < 0.0)
  {
    tempAngle += M_PI;
  }

  target[1] = tempAngle;
}
void convertCart(double *source, double *target)
{
  target[0] = source[0] * cos(source[1]);
  target[1] = source[0] * sin(source[1]);
}
double encoderToRad(int count)
{
  return ((double) count / (double) ticksPerRotation) * 2 * M_PI;
}
double degToRad(double degrees)
{
  return M_PI * (degrees / 180);
}
