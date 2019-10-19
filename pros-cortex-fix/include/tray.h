#ifndef TRAY_H
#define TRAY_H

/**
 * Sets the motor power for the tray
 *
 * @param power - the power to set the motor to
 */
void setTray(int power);
/**
 * Stops the tray motor
 */
void stopTray();
/**
 * Moves the tray to the vertical position for scoring
 */
void moveTrayVertical();
/**
 * Moves the tray to the angled position for cube intake
 */
void moveTrayAngled();

#endif
