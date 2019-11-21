#ifndef TRAY_H
#define TRAY_H

#define TRAY_ANGLED 40
#define TRAY_VERTICAL 1450
#define TRAY_ARM 700

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
/**
 * Gets the value from the tray potentiometer
 *
 * @returns the value from the tray pot
 */
int getTrayPot();

#endif
