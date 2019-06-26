#ifndef VELOCITY_CONTROL_H
#define VELOCITY_CONTROL_H

/**
 * Task that handles PID control of left and right velocities
 */
void velocityController(void *ignore);
/**
 * initializes the velocity controller
 */
void initializeVelocityController();
/**
 * Stops the velocity controller and all drive motors
 */
void stopVelocityController();
/**
 * Sets the target left and right wheel velocities
 *
 * @param leftV - the target left wheel linear velocity in inches per second
 * @param rightV - the target right wheel linear velocity in inches per second
 */
void setTargetVelocity(double leftV, double rightV);

#endif
