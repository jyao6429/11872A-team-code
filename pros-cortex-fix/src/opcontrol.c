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
TaskHandle test;

void testVControl(void *ignore)
{
	initializeVelocityController();

	setTargetVelocity(12.0, 12.0);
}
void testChassis(void *ignore)
{
	resetPositionFull(&globalPose, 0.0, 0.0, 0.0, false);
	// Test turning PID
	turnToAngle(90.0, 100, true, true);
	turnToAngle(0.0, 100, true, false);

	// Test driveStraightToPose PID
	//driveStraightToPoint(0.0, 24.0, 70, true);
	//driveStraightToPoint(48.0, 48.0, 70, true);
	//driveStraightToPoint(48.0, 0.0, 70, true);
	//driveStraightToPoint(0.0, 0.0, 70, true);

	// Test driveToPose PID
	//driveToPose(0.0, 24.0, 45.0, 100, false, true);
	//driveToPose(0.0, 48.0, -10.0, 100, true, true);
	//driveToPose(48.0, -48.0, -90.0, 70, true, true);

	// Test driveAlongLineToPose
	//driveAlongLineToPose(-24.0, 24.0, -45.0, 100, true, true);
	//driveAlongLineToPose(-48.0, 48.0, -90.0, 100, true, true);
	//driveAlongLineToPose(2.0, 12.0, 0.0, 100, true, true);

	// Test driveAndParkToPose
	//driveToPose(-24.0, 48.0, -100.0, 100, false, true);
	//driveToPose(-24.0, 0.0, 90.0, 100, false, true);
	//driveAndParkToPose(0.0, 0.0, 0.0, 100, true, true);

	printf("Done Testing\n");
	stopMotors();
}
void gatherVelocityData(void *ignore)
{
	resetPositionFull(&globalPose, 0.0, 0.0, 0.0, true);

	int powerDiff = -150;
	unsigned long timer = millis();

	while (powerDiff <= 0)
	{
		powerMotorsLinear(127 + powerDiff, 127);

		double angularV = globalVel.angle;

		// Debug
		logDataInt("powerDiff", powerDiff);
		logDataDouble("angularV80", angularV * 80);
		logDataDouble("angularV70", angularV * 70);
		logDataDouble("angularVDeg", radToDeg(angularV));

		if (millis() - timer > 500)
		{
			powerDiff += 15;
			timer = millis();
		}
		delay(40);
	}
	stopMotors();
}
void testNew(void *ignore)
{
	resetPositionFull(&globalPose, 0.0, 0.0, 0.0, true);
	resetVelocity(&globalVel, globalPose);

	//turnToTargetNew(-12, 0, TURN_CCW, 0.5, 25, 12, 0.0, true, true);

	//sweepTurnToTarget(12.0, 12.0, 90.0, 12, TURN_CW, 127, true, true);

	moveToTargetSimple(0.0, 36.0, 0.0, 0.0, 127, 127, 1, 0, 0, 0, STOP_NONE, MTT_SIMPLE);
	moveToTargetSimple(-36, 72, 0, 36, 127, 127, 0.5, 0, 50, 0, STOP_HARSH, MTT_CASCADING);

	while (false)
	{
		printf("X: %3.3f   Y: %3.3f   A: %3.3f\n", globalPose.x, globalPose.y, radToDeg(globalPose.angle));
		delay(50);
	}
	printf("Done Testing\n");
}
void operatorControl()
{
	printf("Hello PROS User!\n");

	while (1)
	{
		if (digitalRead(PORT_startTestButton) == LOW)
		{
			test = taskCreate(testNew, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT + 2);
			delay(500);
		}
		if (digitalRead(PORT_stopTestButton) == LOW)
		{
			taskDelete(test);
			stopMotors();
			printf("Stopped Testing\n");
			delay(500);
		}

		int leftPower = joystickGetAnalog(1, 3);
		int rightPower = joystickGetAnalog(1, 2);

		// Deadband for joysticks
		if (abs(leftPower) < 15)
			leftPower = 0;
		if (abs(rightPower) < 15)
			rightPower = 0;

		//powerMotors(leftPower, rightPower);

		// Debug APS
		mutexTake(mutexes[MUTEX_POSE], 10);
		//printf("X: %f\tY: %f\tANGLE: %f\n", robotPose[POSE_X], robotPose[POSE_Y], radToDeg(robotPose[POSE_ANGLE]));
		mutexGive(mutexes[MUTEX_POSE]);

		// Return to origin when button pressed
		if (joystickGetDigital(1, 8, JOY_UP))
		{
			//driveStraightToPose(0.0, 0.0, 0.0, 70, true, false);
		}

		delay(20);
	}
}
