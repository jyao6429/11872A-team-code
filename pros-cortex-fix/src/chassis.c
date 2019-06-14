#include "main.h"

void powerMotors(int leftPower, int rightPower)
{
  motorSet(PORT_leftBackMotor, leftPower);
  motorSet(PORT_leftFrontMotor, leftPower);
  motorSet(PORT_rightBackMotor, -rightPower);
  motorSet(PORT_rightFrontMotor, -rightPower);
}

// Enums for the various PID controllers
enum PIDControllers
{
  PID_STRAIGHT,
  PID_ROTATE,
};
// Array of PID controllers for various motions
PID controllers[2];

void turnToAngle(double targetAngle, int maxSpeed, boolean isAccurate)
{
  
}
