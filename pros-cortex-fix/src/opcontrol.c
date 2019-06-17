/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

/*
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
void operatorControl()
{
	printf("Hello PROS User!\n");

	// Test turning PID
	//turnToAngle(90, 35, true, true);

	// Test driveStraightToPose PID

	//driveStraightToPose(0.0, 48.0, -90.0, 70, true);
	//driveStraightToPose(48.0, 48.0, 0.0, 70, true);
	//driveStraightToPose(48.0, 0.0, 90.0, 70, true);
	//driveStraightToPose(0.0, 0.0, 180.0, 70, true);


	while (1)
	{
		int leftPower = joystickGetAnalog(1, 3);
		int rightPower = joystickGetAnalog(1, 2);

		// Deadband for joysticks
		if (abs(leftPower) < 15)
			leftPower = 0;
		if (abs(rightPower) < 15)
			rightPower = 0;

		powerMotors(leftPower, rightPower);

		// Debug APS
		mutexTake(mutexes[MUTEX_POSE], 10);
		printf("X: %f\tY: %f\tANGLE: %f\n", robotPose[POSE_X], robotPose[POSE_Y], radToDeg(robotPose[POSE_ANGLE]));
		mutexGive(mutexes[MUTEX_POSE]);

		if (joystickGetDigital(1, 8, JOY_UP))
		{
			driveStraightToPose(0.0, 0.0, 0.0, 70, false);
		}

		delay(20);
	}
}
