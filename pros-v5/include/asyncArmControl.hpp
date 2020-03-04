#ifndef ASYNC_ARM_CONTROL_H
#define ASYNC_ARM_CONTROL_H

#include "main.h"

#define ARM_ZERO 0
#define ARM_SECOND 90
#define ARM_LOW 340
#define ARM_MED 560

// The next move to be performed by the robot
extern int nextArmTarget;

/**
 * Initializes the arm motor and controller
 */
void initArm();
/**
 * Starts the async controller and resets variables to defaults
 */
void startAsyncArmController();
/**
 * Stops the async controller and resets variables to defaults
 */
void stopAsyncArmController();
bool waitUntilArmMoveComplete(int timeout);
/**
 * Waits until the arm completes the current motion
 */
void waitUntilArmMoveComplete();
/**
 * Moves intake arms to the designated target position
 *
 * @param armTarget - the target position for the arms
 */
void moveArmsToPosition(int armTarget);
/**
 * Moves intake arms to the down position for cube intake
 */
void moveArmsZeroAsync();
/**
 * Holds intake arms to intake a cube stacked on one other cube
 */
void moveArmsSecondAsync();
/**
 * Holds intake arms to score/descore low towers
 */
void moveArmsLowAsync();
/**
 * Holds intake arms to score/descore medium towers
 */
void moveArmsMedAsync();
/**
 * Resets the IME for the arm
 */
void resetArm();
/**
 * Returns if the tray needs to be tilted forward for the arms to clears
 *
 * @return true if tray needs to be forward, false if not
 */
bool needsTrayOverride();
/**
 * Returns the arm IME value
 *
 * @return the arm position
 */
double getArmPosition();
/**
 * Powers the motors on the intake arms
 *
 * @param power - (-1.0 - 1.0) the power for the arms, positive values raise the arm
 */
void setArms(double power);
/**
 * Powers the motors on the intake arms
 *
 * @param power - (-127 - 127) the power for the arms, positive values raise the arm
 */
void setArms(int power);
/**
 * Sets the motor velocity for the intake arms
 *
 * @param speed - (-100 - 100) the velocity to set the motor to
 */
void setArmsVel(double speed);
/**
 * Powers the motors on the intake rollers
 *
 * @param power - (-1.0 - 1.0) the power for the rollers, positive values intake cubes
 */
void setRollers(double power);
/**
 * Powers the motors on the intake rollers
 *
 * @param power - (-127 - 127) the power for the rollers, positive values intake cubes
 */
void setRollers(int power);
/**
 * Sets the motor velocity for the intake rollers
 *
 * @param speed - (-200 - 200) the velocity to set the motor to
 */
void setRollersVel(double speed);
/**
 * Stops the intake arms
 */
void stopArms();
/**
 * Stops the intake rollers
 */
void stopRollers();
#endif
