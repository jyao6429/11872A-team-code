#include "main.h"

void turnToAngleNew(double targetAngle, TurnDir turnDir, double fullPowerRatio, int coastPower, double angleError, bool harshStop, bool isDegrees)
{
  // Convert to radians if needed
  if (isDegrees)
    targetAngle = degToRad(targetAngle);

  // Calculate correct turn direction if not given
  if (turnDir == TURN_CH)
  {
    if (fmod(targetAngle - globalPose.angle, M_PI * 2) > M_PI)
      turnDir = TURN_CCW;
    else
      turnDir = TURN_CW;
  }

  // Variable to store the angle to stop turning at full power
  double endFullPower;

  // Switch between the turn directions
  switch (turnDir)
  {
    case TURN_CW:
      // Convert targetAngle to be nearest equivalent greater than the current robot orientation
      targetAngle = globalPose.angle + fmod(targetAngle - globalPose.angle, M_PI * 2);
      // Calculate angle to stop going at full power
      endFullPower = globalPose.angle * (1 - fullPowerRatio) + targetAngle * fullPowerRatio;
      // Set motors to full power
      powerMotors(127, -127);

      // Wait until past endFullPower
      while (globalPose.angle < endFullPower)
      {
        delay(10);
      }
      // Now set to coastPower
      powerMotors(coastPower, -coastPower);

      // Wait until within error range
      while (globalPose.angle < targetAngle - degToRad(angleError))
      {
        delay(10);
      }

      // Now stop the robot according to parameters
      if (harshStop)
      {
        powerMotors(-20, 20);
        delay(150);
      }
      stopMotors();

      break;
    case TURN_CCW:
      // Convert targetAngle to be nearest equivalent greater than the current robot orientation
      targetAngle = globalPose.angle - fmod(globalPose.angle - targetAngle, M_PI * 2);
      // Calculate angle to stop going at full power
      endFullPower = globalPose.angle * (1 - fullPowerRatio) + targetAngle * fullPowerRatio;
      // Set motors to full power
      powerMotors(-127, 127);

      // Wait until past endFullPower
      while (globalPose.angle > endFullPower)
      {
        delay(10);
      }
      // Now set to coastPower
      powerMotors(-coastPower, coastPower);

      // Wait until within error range
      while (globalPose.angle > targetAngle + degToRad(angleError))
      {
        delay(10);
      }

      // Now stop the robot according to parameters
      if (harshStop)
      {
        powerMotors(20, -20);
        delay(150);
      }
      stopMotors();
      break;
  }
  // Log
  printf("TA: %3.3f   X: %3.3f   Y:%3.3f   A: %3.3f", radToDeg(targetAngle), globalPose.x, globalPose.y, radToDeg(globalPose.angle));
}
void applyHarshStop()
{
  // Get velocity vector of the robot
  Cart vel;
  vel.x = globalVel.x;
  vel.y = globalVel.y;

  // Convert to polar vector, the shift the angle so facing along robot
  Polar polarVel;
  cartToPolar(vel, &polarVel);
  polarVel.angle += globalPose.angle;
  // Shift back to cartesian vectors
  polarToCart(polarVel, &vel);

  // Get the powers for y and angle
  double yPower = vel.y;
  double anglePower = globalVel.angle;

  // Debug
  printf("yVel: %3.3f   aVel: %3.3f\n", yPower, anglePower);

  // Convert from velocities to power (WILL NEED TO TUNE THESE CONSTANTS)
  yPower *= -0.7;
  anglePower *= -6.3;

  // Initial calculation of left and right wheel powers
  double leftPower = yPower + anglePower;
  double rightPower = yPower - anglePower;

  // Make sure there is a minimum power of 7 (probably need to change that)
  leftPower = copysign(1.0, leftPower) * fmax(fabs(leftPower), 7);
  rightPower = copysign(1.0, rightPower) * fmax(fabs(rightPower), 7);

  // Make sure power is below maxiumum of 30 (probably need to change this as well)
  int leftPowInt = copysign(1.0, leftPower) * (fabs(leftPower) > 30 ? 30 : fabs(leftPower));
  int rightPowInt = copysign(1.0, rightPower) * (fabs(rightPower) > 30 ? 30 : fabs(rightPower));

  // Debug
  printf("Applying harsh stop: LP: %d   RP: %d\n", leftPowInt, rightPowInt);

  // Drive the motors
  powerMotors(leftPowInt, rightPowInt);
  delay(150);
  stopMotors();
}
