#ifndef ASYNC_ARM_CONTROL_H
#define ASYNC_ARM_CONTROL_H

#include "main.h"

#define ARM_ZERO 0
#define ARM_SCORE 1000
#define ARM_LOW 1630
#define ARM_MED 2300

// The next move to be performed by the robot
int nextArmTarget;

void initArm();
void resetArm();
bool needsTrayOverride();
bool needsTrayOverride();
double getArmPosition();
/**
 * Waits until the arm completes the current motion
 */
void waitUntilArmMoveComplete();
/**
 * Starts the async controller and resets variables to defaults
 */
void startAsyncArmController();
/**
 * Stops the async controller and resets variables to defaults
 */
void stopAsyncArmController();
/**
 * Moves intake arms to the down position for cube intake
 */
void moveArmsZeroAsync();
/**
 * Holds intake arms to clear the intake lock in order to score in the goal zones
 */
void moveArmsScoreAsync();
/**
 * Holds intake arms to score/descore low towers
 */
void moveArmsLowAsync();
/**
 * Holds intake arms to score/descore medium towers
 */
void moveArmsMedAsync();
/**
 * Gets a normalized value from the arm potentiometer
 *
 * @returns the normalized value from the arm pot
 */
int getArmPot();
/**
 * Powers the motors on the intake arms
 *
 * @param power - the power for the arms, positive values raise the arm
 */
void setArms(int power);
/**
 * Powers the motors on the intake rollers
 *
 * @param power - the power for the rollers, positive values intake cubes
 */
void setRollers(int power);
/**
 * Stops the intake arms
 */
void stopArms();
/**
 * Stops the intake rollers
 */
void stopRollers();
#endif
