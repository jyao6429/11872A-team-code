#include "main.h"

// Physical parameters in inches
const double sL = 7.0625;               // distance from center to left tracking wheel
const double sR = 7.0625;               // distance from center to right tracking wheel
const double sB = 10.5;                 // distance from center to back tracking wheel
const double sideWheelDiameter = 4.0;   // diameter of side wheels
const double backWheelDiameter = 4.0;   // diameter of back wheel
// Encoder counts
const int sideEncoderResolution = 360;   // side encoder ticks per 360 degrees of motion
const int backEncoderResolution = 360;   // back encoder ticks per 360 degrees of motion

// Current angle calculated by gyro
double gyroAngle = 0.0;

// Previous positions
double prevPos[2] = {0.0, 0.0};
double prevAngle = 0.0;
double resetAngle = 0.0;
// Previous encoder values
int prevLeft = 0;
int prevRight = 0;
int prevBack = 0;

// Not sure if using v5 or cortex, just rewrite these functions to make code work
int getLeftEncoder()
{
  //return encoderGet(leftEncoder);
  return 0;
}
int getRightEncoder()
{
  return encoderGet(rightEncoder);
}
int getBackEncoder()
{
  return encoderGet(backEncoder);
}
void resetLeftEncoder()
{
  //encoderReset(leftEncoder);
  prevLeft = 0;
}
void resetRightEncoder()
{
  encoderReset(rightEncoder);
  prevRight = 0;
}
void resetBackEncoder()
{
  encoderReset(backEncoder);
  prevBack = 0;
}
void initializeAPS(double startX, double startY, double startAngle)
{
  // Reset to starting positions
  resetPosition(startX, startY, startAngle);

  // Create new tasks to track position
  taskCreate(startGyroIntegral, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT + 2);
  taskCreate(startTracking, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT + 1);
}
void resetPosition(double resetX, double resetY, double resetAngle)
{
  // Reset all the encoders
  resetLeftEncoder();
  resetRightEncoder();
  resetBackEncoder();

  // Set reset positions
  prevPos[X_COMP] = resetX;
  prevPos[Y_COMP] = resetY;
  // Set reset orientation
  resetAngle = degToRad(resetAngle);
  gyroAngle = resetAngle;
  prevAngle = resetAngle;
}
void startGyroIntegral(void *ignore)
{
  unsigned long gyroTimer = millis();

  while (true)
  {
    // Calculate delta time from last iteration
    double deltaTime = ((double) (millis() - gyroTimer)) / 1000;

    // Reset loop timer
    gyroTimer = millis();

    // Add to angle with rate from gyro
    gyroAngle += degToRad(gyro_get_rate(gyro) * deltaTime);

    delay(1);
  }
}
void startTracking(void *ignore)
{
  while (true)
  {
    // Get current encoder values
    //int currentLeft = getLeftEncoder();
    int currentRight = getRightEncoder();
    int currentBack = getBackEncoder();

    // Calculate traveled distance in inches
    //double deltaLeft = calculateTravelDistance(currentLeft - prevLeft, sideWheelDiameter, sideEncoderResolution);
    double deltaRightDistance = calculateTravelDistance(currentRight - prevRight, sideWheelDiameter, sideEncoderResolution);
    double deltaBackDistance = calculateTravelDistance(currentBack - prevBack, backWheelDiameter, backEncoderResolution);

//printf("%f\t%f\t%f\n", deltaLeft, deltaRight, deltaBack);

    // Update prev values;
    //prevLeft = currentLeft;
    prevRight = currentRight;
    prevBack = currentBack;

    // Calculate total change since last reset
    //double totalLeftDistance = calculateTravelDistance(currentLeft, sideWheelDiameter, sideEncoderResolution);
    //double totalRightDistance = calculateTravelDistance(currentRight, sideWheelDiameter, sideEncoderResolution);

    // Calculate new absolute orientation
    //double newAngle = resetAngle + (totalLeftDistance - totalRightDistance) / (sL + sR);

    // Calculate change in angle
    //double deltaAngle = newAngle - prevAngle;
    double deltaAngle = gyroAngle - prevAngle;

    // Calculate local offset
    double localOffset[] = {0.0, 0.0};

    // If drove straight (about < 1 deg diff)
    if (fabs(deltaAngle) < 0.01)
    {
      localOffset[X_COMP] = deltaBackDistance;
      localOffset[Y_COMP] = deltaRightDistance;
    }
    else
    {
      localOffset[X_COMP] = 2 * sin(deltaAngle / 2) * ((deltaBackDistance / deltaAngle) + sB);
      localOffset[Y_COMP] = 2 * sin(deltaAngle / 2) * ((deltaRightDistance / deltaAngle) + sR);
    }

    // Calculate average angle
    double avgAngle = prevAngle + (deltaAngle / 2);

    // Convert localOffset to polar
    double localPolar[] = {0.0, 0.0};
    convertPolar(localOffset, localPolar);

    // Shift angle
    localPolar[ANGLE] -= avgAngle;

    // Converting back to cartesian gives the globalOffset
    double globalOffset[] = {0.0, 0.0};
    convertCart(localPolar, globalOffset);

    // Calculate new absolute position
    prevPos[X_COMP] += globalOffset[X_COMP];
    prevPos[Y_COMP] += globalOffset[Y_COMP];

    // Update previous angle
    //prevAngle = newAngle;
    prevAngle += deltaAngle;

    delay(2);
  }
}
double nearestEquivalentAngle(double ref, double target)
{
  return round((ref - target) / (2 * M_PI)) * 2 * M_PI + target;
}
void convertPolar(double *source, double *target)
{
  // Calculates magnitude of vector with distance formula
  target[MAGNITUDE] = sqrt(pow(source[X_COMP], 2) + pow(source[Y_COMP], 2));
  // Calculates angle with arctan, automatically gives angle in correct quadrant
  target[ANGLE] = atan2(source[Y_COMP], source[X_COMP]);
}
void convertCart(double *source, double *target)
{
  // Calculate x component with cosine
  target[X_COMP] = source[MAGNITUDE] * cos(source[ANGLE]);
  // Calculate y component with sine
  target[Y_COMP] = source[MAGNITUDE] * sin(source[ANGLE]);
}
double calculateTravelDistance(int encoderCount, double wheelDiameter, int encoderResolution)
{
  return ((double) encoderCount * M_PI * wheelDiameter) / ((double) encoderResolution);
}
double degToRad(double degrees)
{
  return degrees * (M_PI / 180);
}
double radToDeg(double rads)
{
  return rads * (180 / M_PI);
}
