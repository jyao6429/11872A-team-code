#include "main.h"

void sweepTurnToTarget(double targetX, double targetY, double targetAngle, double targetRadius, TurnDir turnDir, int power, bool slowPark, bool isDegrees)
{
  // Convert targetAngle to radians if needed
  if (isDegrees)
    targetAngle = degToRad(targetAngle);

  // Vectors for position calculations
  Cart cartVector;
  Polar polarVector;

  // Calculate correct turn direction if not given
  if (turnDir == TURN_CH)
  {
    cartVector.x = globalPose.x - targetX;
    cartVector.y = globalPose.y - targetY;
    cartToPolar(cartVector, &polarVector);
    polarVector.angle += targetAngle;
    polarToCart(polarVector, &cartVector);
    turnDir = cartVector.x > 0 ? TURN_CW : TURN_CCW;
  }

  // The x and y coordinates of the center of the circle the robot is going to follow
  double xOrigin, yOrigin;
  // The velocities of the robot
  double linearV, angularV, prevAngluarV = 0.0;
  // The relative radius and angle to the center of the circle
  double localRadius, localAngle;

  // Constant gains (probably need to tune these)

  // localRadius vs targetRadius
  const double kR = 15.0;
  // localAngle vs robot angle
	const double kA = 5.0;
  // base angularV to power
	const double kB = 60.0;
  // proportional angularV
	const double kP = 30.0;
  // derivative angularV
	const double kD = 2000.0;

  // Cycle variables
  unsigned long cycleTime = millis();
  const int dT = 40;

  // Switch between turning clockwise or counterclockwise
  switch (turnDir)
  {
    case TURN_CW:
      // Calculate the coordinates of the center of the circle
      cartVector.y = 0;
      cartVector.x = targetRadius;
      cartToPolar(cartVector, &polarVector);
      polarVector.angle -= targetAngle;
      polarToCart(polarVector, &cartVector);
      yOrigin = targetY + cartVector.y;
      xOrigin = targetX + cartVector.x;

      // Calculate the angle to the circle
      localAngle = atan2(globalPose.x - xOrigin, globalPose.y - yOrigin);

      // Find nearestEquivalentAngle of targetAngle, rotating by 180 degrees if going backwards
      targetAngle = nearestEquivalentAngle(targetAngle, power > 0 ? globalPose.angle : (globalPose.angle + M_PI));

      // Make the move, keeping track of cycle
      cycleTime = millis();
      do
      {
        // Get the global orientation of the robot
        double globalAngle = globalPose.angle;
        if (power < 0)
          globalAngle += M_PI;

        // Relative distance to origin
        double deltaY = globalPose.y - yOrigin;
        double deltaX = globalPose.x - xOrigin;

        // Do local calculations again
        localRadius = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
        localAngle = nearestEquivalentAngle(atan2(deltaX, deltaY), localAngle);

        // Get and calculate velocities of the robot
        angularV = globalVel.angle;
        linearV = globalVel.x * sin(localAngle + M_PI / 2) + globalVel.y * cos(localAngle + M_PI / 2);

        // Calculate target angular velocity of the robot, based on the various errors
                          // The angular velocity needed to hold the curve based on the tangential velocity, with a minimum of 15 in/s
        double targetOmega = fmax(linearV, 15) / localRadius
                          // The natural log of the percent error of the current radius verses the target radius
                           + kR * log(localRadius / targetRadius)
                          // The error of the angle the robot is facing verses the required angle to complete the turn
                           + kA * (nearestEquivalentAngle(localAngle + M_PI / 2, globalAngle) - globalAngle);

        // Calculate the differential power to follow the curve
                               // The expected power needed for this target
        int turnPowerDiff = round(kB * targetOmega
                               // The PD controller for angularV
                                + kP * (targetOmega - angularV) + kD * (prevAngluarV - angularV) / (double) dT);

        // Update previous angularV
        prevAngluarV = angularV;

        // Clamp the difference from 0 - 150
        if (turnPowerDiff < 0)
          turnPowerDiff = 0;
        else if (turnPowerDiff > 150)
          turnPowerDiff = 150;

        // Depends on if driving forward or backwards, power the motors
        if (power > 0)
          powerMotors(power, power - turnPowerDiff);
        else
          powerMotors(power + turnPowerDiff, power);

        // Make sure loops with correct cycle
        taskDelayUntil(&cycleTime, dT);

        // Loop while angle error is greater than the target margin
      } while ((power > 0 ? globalPose.angle : (globalPose.angle + M_PI)) - targetAngle < (slowPark ? -0.1 : -0.15));
      break;
    case TURN_CCW:
      // Calculate the coordinates of the center of the circle
      cartVector.y = 0;
      cartVector.x = targetRadius;
      cartToPolar(cartVector, &polarVector);
      polarVector.angle += targetAngle;
      polarToCart(polarVector, &cartVector);
      yOrigin = targetY + cartVector.y;
      xOrigin = targetX + cartVector.x;

      // Calculate the angle to the circle
      localAngle = atan2(globalPose.x - xOrigin, globalPose.y - yOrigin);

      // Find nearestEquivalentAngle of targetAngle, rotating by 180 degrees if going backwards
      targetAngle = nearestEquivalentAngle(targetAngle, power > 0 ? globalPose.angle : (globalPose.angle + M_PI));

      // Make the move, keeping track of cycle
      cycleTime = millis();
      do
      {
        // Get the global orientation of the robot
        double globalAngle = globalPose.angle;
        if (power < 0)
          globalAngle += M_PI;

        // Relative distance to origin
        double deltaY = globalPose.y - yOrigin;
        double deltaX = globalPose.x - xOrigin;

        // Do local calculations again
        localRadius = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
        localAngle = nearestEquivalentAngle(atan2(deltaX, deltaY), localAngle);

        // Get and calculate velocities of the robot
        angularV = globalVel.angle;
        linearV = globalVel.x * sin(localAngle - M_PI / 2) + globalVel.y * cos(localAngle - M_PI / 2);

        // Calculate target angular velocity of the robot, based on the various errors
                          // The angular velocity needed to hold the curve based on the tangential velocity, with a minimum of 15 in/s
        double targetOmega = -fmax(linearV, 15) / localRadius
                          // The natural log of the percent error of the current radius verses the target radius
                           + kR * log(targetRadius / localRadius)
                          // The error of the angle the robot is facing verses the required angle to complete the turn
                           + kA * (nearestEquivalentAngle(localAngle - M_PI / 2, globalAngle) - globalAngle);

        // Calculate the differential power to follow the curve
                               // The expected power needed for this target
        int turnPowerDiff = round(kB * targetOmega
                               // The PD controller for angularV
                                + kP * (targetOmega - angularV) + kD * (prevAngluarV - angularV) / (double) dT);

        // Update previous angularV
        prevAngluarV = angularV;

        // Clamp the difference from 0 - -150
        if (turnPowerDiff > 0)
          turnPowerDiff = 0;
        else if (turnPowerDiff < -150)
          turnPowerDiff = -150;

        // Depends on if driving forward or backwards, power the motors
        if (power > 0)
          powerMotors(power + turnPowerDiff, power);
        else
          powerMotors(power, power - turnPowerDiff);

        // Make sure loops with correct cycle
        taskDelayUntil(&cycleTime, dT);

        // Loop while angle error is greater than the target margin
      } while ((power > 0 ? globalPose.angle : (globalPose.angle + M_PI)) - targetAngle > (slowPark ? 0.1 : 0.15));
      break;
  }
  stopMotors();
  // Log
  printf("(sweepTurnToTarget)   TX: %3.3f   TY: %3.3f   TA: %3.3f   X: %3.3f   Y: %3.3f   A: %3.3f\n", targetX, targetY, targetAngle, globalPose.x, globalPose.y, radToDeg(globalPose.angle));
}
void turnToAngleNew(double targetAngle, TurnDir turnDir, double fullPowerRatio, int coastPower, double stopPowerDiff, bool harshStop, bool isDegrees)
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
      while (globalPose.angle < targetAngle - degToRad(stopPowerDiff))
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
      // Convert targetAngle to be nearest equivalent less than the current robot orientation
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
      while (globalPose.angle > targetAngle + degToRad(stopPowerDiff))
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
  printf("(turnToAngleNew)   TA: %3.3f   X: %3.3f   Y:%3.3f   A: %3.3f", radToDeg(targetAngle), globalPose.x, globalPose.y, radToDeg(globalPose.angle));
}
void turnToTargetNew(double targetX, double targetY, TurnDir turnDir, double fullPowerRatio, int coastPower, double stopPowerDiff, double angleOffset, bool harshStop)
{
  // Calculate correct turn direction if not given
  if (turnDir == TURN_CH)
  {
    if (fmod(atan2(targetX - globalPose.x, targetY - globalPose.y) + angleOffset - globalPose.angle, M_PI * 2) > M_PI)
      turnDir = TURN_CCW;
    else
      turnDir = TURN_CW;
  }

  // Variable to store the angle to stop turning at full power
  double endFullPower, targetAngle;

  // Switch between the turn directions
  switch (turnDir)
  {
    case TURN_CW:
      // Calculate targetAngle to be nearest equivalent angle greater than the current robot orientation
      targetAngle = globalPose.angle + fmod(atan2(targetX - globalPose.x, targetY - globalPose.y) + angleOffset - globalPose.angle, M_PI * 2);
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
      while (globalPose.angle < nearestEquivalentAngle(atan2(targetX - globalPose.x, targetY - globalPose.y) + angleOffset, targetAngle) - degToRad(stopPowerDiff))
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
      // Calculate targetAngle to be nearest equivalent angle less than the current robot orientation
      targetAngle = globalPose.angle - fmod(globalPose.angle - atan2(targetX - globalPose.x, targetY - globalPose.y) - angleOffset, M_PI * 2);
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
      while (globalPose.angle > nearestEquivalentAngle(atan2(targetX - globalPose.x, targetY - globalPose.y) + angleOffset, targetAngle) + degToRad(stopPowerDiff))
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
  printf("(turnToTargetNew)   TXF: %3.3f   TYF: %3.3f   TA: %3.3f   X: %3.3f   Y:%3.3f   A: %3.3f", targetX, targetY, radToDeg(targetAngle), globalPose.x, globalPose.y, radToDeg(globalPose.angle));
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
