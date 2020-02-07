#include "main.h"

// Maxiumum achievable linear velocity by the robot in inches per second
#define MAX_LINEAR_VEL 40.0

void moveToTargetSimple(double targetX, double targetY, double startX, double startY, int power, int startPower, double maxErrorX, double decelEarly, int decelPower, double dropEarly, StopType stopType, MTTMode mode)
{
  // Set current target
  lastTarget.x = targetX;
  lastTarget.y = targetY;

  // Create the line to follow
  Line followLine;

  // Set start and end points
  followLine.p1.x = startX;
  followLine.p1.y = startY;
  followLine.p2.x = targetX;
  followLine.p2.y = targetY;

  // Get line info
  double lineLength = getLengthOfLine(followLine);
  double lineAngle = getAngleOfLine(followLine);

  // Get angle to follow, reversing if needed
  double pidAngle = nearestEquivalentAngle(lineAngle - (power < 0 ? M_PI : 0), globalPose.angle);

  // Vectors for current position relative to the ending point
  Cart currentPosCart;
  Polar currentPosPolar;

  // Store sine and cosine of the lineAngle
  double lineSin = sin(lineAngle);
  double lineCos = cos(lineAngle);
  // Stores the robot's linear velocity along the direction of the line
  double currentVel = 0.0;

  // Stores the last applied power
  int prevPower = startPower;
  // The amount to correct for deviation from the line
  double angleCorrection = 0.0;

  // Just start driving if mode is simple
  if (mode == MTT_SIMPLE)
    setDrive(power, power);

  // The final power applied to the wheels after all calculations
  int finalPower = power;

  // Variable for cycling
  std::uint32_t cycleTime = pros::millis();
  int dT = 10;

  do
  {
    // Calculate relative position to ending point
    currentPosCart.x = globalPose.x - targetX;
    currentPosCart.y = globalPose.y - targetY;
    // Rotate vector so it lines up with line
    cartToPolar(currentPosCart, &currentPosPolar);
    currentPosPolar.angle += lineAngle;
    polarToCart(currentPosPolar, &currentPosCart);

    // Calculate deviations from line
    if (maxErrorX != 0.0)
    {
      // Error from not following line straight
      double errAngle = globalPose.angle - pidAngle;
      // Error from deviation from line
      double errX = currentPosCart.x + currentPosCart.y * tan(errAngle);
      // The correct angle to face if going straight to target points
      double correctAngle = atan2(targetX - globalPose.x, targetY - globalPose.y);
      // Correct if going backwards
      if (power < 0)
        correctAngle += M_PI;
      // Calculate the needed angleCorrection (NEED TO TUNE CONSTANT)
      angleCorrection = (fabs(errX) > maxErrorX) ? 8.0 * (nearestEquivalentAngle(correctAngle, globalPose.angle) - globalPose.angle) * copysign(1.0, power) : 0.0;
    }

    // Switch between the modes
    if (mode != MTT_SIMPLE)
    {
      switch (mode)
      {
        case MTT_PROPORTIONAL:
          // Simply a P controller (NEED TO TUNE CONSTANT)
          finalPower = round((-127.0 / 40.0) * currentPosCart.y * copysign(1.0, power));
          break;
        case MTT_CASCADING:;
          // Need to tune these constants
          double kB, kP;
          if (false)
          {
            kB = 5.0;
            kP = 3.2;
          }
          else
          {
            kB = 4.5;
            kP = 2.5;
          }

          // Calculate target velocity (what does this mean?)
          double vTarget = MAX_LINEAR_VEL * (1 - exp(0.07 * (currentPosCart.y + dropEarly)));
          finalPower = round((kB * vTarget + kP * (vTarget - currentVel)) * copysign(1.0, power));
          break;
      }
      // Clamp finalPower from 30 to power
      if (abs(finalPower) > abs(power))
        finalPower = power;
      else if (abs(finalPower) < 30)
        finalPower = 30 * copysign(1.0, power);

      // Calculate deltaPower, clamping it to +-5
      int deltaPower = finalPower - prevPower;
      if (abs(deltaPower) > 5)
        deltaPower = 5 * copysign(1.0, deltaPower);

      finalPower = prevPower += deltaPower;
    }

    // Power the motors based on direction of angleCorrection
    switch ((int) copysign(1.01, angleCorrection))
    {
      case 0:
        setDrive(finalPower, finalPower);
        break;
      case 1:
        setDrive(finalPower, (int) (finalPower * exp(-angleCorrection)));
        break;
      case -1:
        setDrive(int (finalPower * exp(angleCorrection)), finalPower);
        break;
    }

    // Update current velocity with respect to the line
    currentVel = lineSin * globalVel.x + lineCos * globalVel.y;

    pros::Task::delay_until(&cycleTime, dT);
  } while(currentPosCart.y < -dropEarly - fmax((currentVel * ((stopType & STOP_SOFT) ? 0.175 : 0.098)), decelEarly));

  // Start decelerating
  setDrive(decelPower, decelPower);

  do
  {
    // Calculate relative position to ending point
    currentPosCart.x = globalPose.x - targetX;
    currentPosCart.y = globalPose.y - targetY;
    // Rotate vector so it lines up with line
    cartToPolar(currentPosCart, &currentPosPolar);
    currentPosPolar.angle += lineAngle;
    polarToCart(currentPosPolar, &currentPosCart);

    // Update current velocity with respect to the line
    currentVel = lineSin * globalVel.x + lineCos * globalVel.y;

    pros::Task::delay_until(&cycleTime, dT);
  } while(currentPosCart.y < -dropEarly - (currentVel * ((stopType & STOP_SOFT) ? 0.175 : 0.098)));

  // Stop with given parameters
  if (stopType & STOP_SOFT)
  {
    setDrive((int) (-6 * copysign(1.0, power)), (int) (-6 * copysign(1.0, power)));
    do
    {
      // Calculate relative position to ending point
      currentPosCart.x = globalPose.x - targetX;
      currentPosCart.y = globalPose.y - targetY;
      // Rotate vector so it lines up with line
      cartToPolar(currentPosCart, &currentPosPolar);
      currentPosPolar.angle += lineAngle;
      polarToCart(currentPosPolar, &currentPosCart);

      // Update current velocity with respect to the line
      currentVel = lineSin * globalVel.x + lineCos * globalVel.y;

      pros::Task::delay_until(&cycleTime, dT);
    } while(currentVel > 7 && currentPosCart.y < 0);
  }

  if (stopType & STOP_HARSH)
    applyHarshStop();
  else
    stopDrive();

  // log
  printf("(moveToTargetSimple) TX: %3.3f   TY: %3.3f   SX: %3.3f   SY: %3.3f   X: %3.3f   Y: %3.3f\n", targetX, targetY, startX, startY, globalPose.x, globalPose.y);
}
void moveToTargetDisSimple(double angle, double distance, double startX, double startY, int power, int startPower, double maxErrorX, double decelEarly, int decelPower, double dropEarly, StopType stopType, MTTMode mode, bool isDegrees)
{
  // Convert angle to radians if needed
  if (isDegrees)
    angle = degToRad(angle);

  moveToTargetSimple(startX + distance * sin(angle), startY + distance * cos(angle), startX, startY, power, startPower, maxErrorX, decelEarly, decelPower, dropEarly, stopType, mode);
}
void sweepTurnToTarget(double targetX, double targetY, double targetAngle, double targetRadius, TurnDir turnDir, int power, bool isAccurate, bool isDegrees)
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
	const double kB = 70.0;
  // proportional angularV
	const double kP = 30.0;
  // derivative angularV
	const double kD = 2000.0;

  // Cycle variables
  std::uint32_t cycleTime = pros::millis();
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
      cycleTime = pros::millis();
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
        double targetOmega = fmax(linearV, 6) / localRadius
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
          setDrive(power, power - turnPowerDiff);
        else
          setDrive(power + turnPowerDiff, power);

        // Debug
        //printf("CAV: %3.3f   CLV: %3.3f   TAV: %3.3f   TPD: %d   LR: %3.3f   LA: %3.3f   TR: %3.3f   GA: %3.3f   TA: %3.3f\n", radToDeg(angularV), linearV, radToDeg(targetOmega), turnPowerDiff, localRadius, radToDeg(localAngle), targetRadius, radToDeg(globalAngle), radToDeg(targetAngle));
        //logDataDouble("localRadius", localRadius);
        //logDataDouble("targetRadius", targetRadius);
        //logDataDouble("localAngle", radToDeg(localAngle + M_PI / 2));
        //logDataDouble("globalAngle", radToDeg(globalAngle));
        //logDataDouble("targetAngle", radToDeg(targetAngle));
        //logDataDouble("targetOmega", radToDeg(targetOmega));
        //logDataInt("turnPowerDiff", turnPowerDiff);
        //logDataDouble("linearV", linearV);
        //logDataDouble("angularV", radToDeg(angularV));

        // Make sure loops with correct cycle
        pros::Task::delay_until(&cycleTime, dT);

        // Loop while angle error is greater than the target margin
      } while ((power > 0 ? globalPose.angle : (globalPose.angle + M_PI)) - targetAngle < (isAccurate ? -0.1 : -0.15));
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
      cycleTime = pros::millis();
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
        double targetOmega = -fmax(linearV, 6) / localRadius
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
          setDrive(power + turnPowerDiff, power);
        else
          setDrive(power, power - turnPowerDiff);

        // Debug
        //printf("CAV: %3.3f   CLV: %3.3f   TAV: %3.3f   TPD: %d   LR: %3.3f   LA: %3.3f   TR: %3.3f   GA: %3.3f   TA: %3.3f\n", radToDeg(angularV), linearV, radToDeg(targetOmega), turnPowerDiff, localRadius, radToDeg(localAngle - M_PI / 2), targetRadius, radToDeg(globalAngle), radToDeg(targetAngle));
        //logDataDouble("localRadius", localRadius);
        //logDataDouble("targetRadius", targetRadius);
        //logDataDouble("localAngle", radToDeg(localAngle - M_PI / 2));
        //logDataDouble("globalAngle", radToDeg(globalAngle));
        //logDataDouble("targetAngle", radToDeg(targetAngle));
        //logDataDouble("targetOmega", radToDeg(targetOmega));
        //logDataInt("turnPowerDiff", turnPowerDiff);
        //logDataDouble("linearV", linearV);
        //logDataDouble("angularV", radToDeg(angularV));

        // Make sure loops with correct cycle
        pros::Task::delay_until(&cycleTime, dT);

        // Loop while angle error is greater than the target margin
      } while ((power > 0 ? globalPose.angle : (globalPose.angle + M_PI)) - targetAngle > (isAccurate ? 0.1 : 0.15));
      break;
  }
  stopDrive();
  // Log
  printf("(sweepTurnToTarget)   TX: %3.3f   TY: %3.3f   TA: %3.3f   X: %3.3f   Y: %3.3f   A: %3.3f\n", targetX, targetY, radToDeg(targetAngle), globalPose.x, globalPose.y, radToDeg(globalPose.angle));
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
      setDrive(127, -127);

      // Wait until past endFullPower
      while (globalPose.angle < endFullPower)
      {
        pros::delay(10);
      }
      // Now set to coastPower
      setDrive(coastPower, -coastPower);

      // Wait until within error range
      while (globalPose.angle < targetAngle - degToRad(stopPowerDiff))
      {
        pros::delay(10);
      }

      // Now stop the robot according to parameters
      if (harshStop)
      {
        setDrive(-20, 20);
        pros::delay(150);
      }
      stopDrive();
      break;
    case TURN_CCW:
      // Convert targetAngle to be nearest equivalent less than the current robot orientation
      targetAngle = globalPose.angle - fmod(globalPose.angle - targetAngle, M_PI * 2);
      // Calculate angle to stop going at full power
      endFullPower = globalPose.angle * (1 - fullPowerRatio) + targetAngle * fullPowerRatio;
      // Set motors to full power
      setDrive(-127, 127);

      // Wait until past endFullPower
      while (globalPose.angle > endFullPower)
      {
        pros::delay(10);
      }
      // Now set to coastPower
      setDrive(-coastPower, coastPower);

      // Wait until within error range
      while (globalPose.angle > targetAngle + degToRad(stopPowerDiff))
      {
        pros::delay(10);
      }

      // Now stop the robot according to parameters
      if (harshStop)
      {
        setDrive(20, -20);
        pros::delay(150);
      }
      stopDrive();
      break;
  }
  // Log
  printf("(turnToAngleNew)   TA: %3.3f   X: %3.3f   Y:%3.3f   A: %3.3f\n", radToDeg(targetAngle), globalPose.x, globalPose.y, radToDeg(globalPose.angle));
}
void turnToTargetNew(double targetX, double targetY, TurnDir turnDir, double fullPowerRatio, int coastPower, double stopPowerDiff, double angleOffset, bool harshStop, bool isDegrees)
{
  // Convert angleOffset to radians if needed
  if (isDegrees)
    angleOffset = degToRad(angleOffset);
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
      setDrive(127, -127);

      // Wait until past endFullPower
      while (globalPose.angle < endFullPower)
      {
        pros::delay(10);
      }
      // Now set to coastPower
      setDrive(coastPower, -coastPower);

      // Wait until within error range
      while (globalPose.angle < nearestEquivalentAngle(atan2(targetX - globalPose.x, targetY - globalPose.y) + angleOffset, targetAngle) - degToRad(stopPowerDiff))
      {
        pros::delay(10);
      }

      // Now stop the robot according to parameters
      if (harshStop)
      {
        setDrive(-20, 20);
        pros::delay(150);
      }
      stopDrive();
      break;
    case TURN_CCW:
      // Calculate targetAngle to be nearest equivalent angle less than the current robot orientation
      targetAngle = globalPose.angle - fmod(globalPose.angle - atan2(targetX - globalPose.x, targetY - globalPose.y) - angleOffset, M_PI * 2);
      // Calculate angle to stop going at full power
      endFullPower = globalPose.angle * (1 - fullPowerRatio) + targetAngle * fullPowerRatio;
      // Set motors to full power
      setDrive(-127, 127);

      // Wait until past endFullPower
      while (globalPose.angle > endFullPower)
      {
        pros::delay(10);
      }
      // Now set to coastPower
      setDrive(-coastPower, coastPower);

      // Wait until within error range
      while (globalPose.angle > nearestEquivalentAngle(atan2(targetX - globalPose.x, targetY - globalPose.y) + angleOffset, targetAngle) + degToRad(stopPowerDiff))
      {
        pros::delay(10);
      }

      // Now stop the robot according to parameters
      if (harshStop)
      {
        setDrive(20, -20);
        pros::delay(150);
      }
      stopDrive();
      break;
  }
  // Log
  printf("(turnToTargetNew)   TXF: %3.3f   TYF: %3.3f   TA: %3.3f   X: %3.3f   Y:%3.3f   A: %3.3f\n", targetX, targetY, radToDeg(targetAngle), globalPose.x, globalPose.y, radToDeg(globalPose.angle));
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
  setDrive(leftPowInt, rightPowInt);
  pros::delay(150);
  stopDrive();
}
