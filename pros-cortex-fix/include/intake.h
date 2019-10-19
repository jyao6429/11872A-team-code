#ifndef INTAKE_H
#define INTAKE_H

#include "main.h"

#define ARM_ZERO 100
#define ARM_LOW_DIFF 1500
#define ARM_MED_DIFF 2500

/**
 * Moves intake arms to score/descore medium towers
 *
 * @param hold - if the arm should be held in that position
 */
void moveArmsMed(bool hold);
/**
 * Moves intake arms to score/descore low towers
 *
 * @param hold - if the arm should be held in that position
 */
void moveArmsLow(bool hold);
/**
 * Moves intake arms to the down position for cube intake
 */
void moveArmsZero();
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
