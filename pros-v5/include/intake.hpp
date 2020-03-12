#ifndef INTAKE_H
#define INTAKE_H

/**
 * Initializes the intake roller motors
 */
void initRollers();

bool waitUntilRollerMoveComplete(int timeout);
void outtakeOneCubeAsync();
void moveRollersRelative(int degrees, double velocity);
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
 * Stops the intake rollers
 */
void stopRollers();

#endif
