#ifndef TILTER_H
#define TILTER_H

/**
 * Sets the motor power for the tilter
 *
 * @param power - the power to set the motor to
 */
void setTilter(int power);
/**
 * Stops the tilter motor
 */
void stopTilter();
/**
 * Moves the tilter to the vertical position for scoring
 */
void moveTilterVertical();
/**
 * Moves the tilter to the angled position for cube intake
 */
void moveTilterAngled();

#endif
