/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

bool isTrayVertical;

void operatorControl()
{
	print("Starting operatorControl\n");

	// Initialize toggle variables
	isTesting = false;
	isTrayVertical = false;

	while (true)
	{
		/*
		// Using interactive button for testing
		if (digitalRead(PORT_interactButton) == LOW)
		{
			if (!isTesting)
			{
				startTesting();
				isTesting = true;
				delay(500);
			}
			else
			{
				stopTesting();
				isTesting = false;
				delay(500);
			}
		}
		*/

		// Hold buttons to set roller speed
		if (joystickGetDigital(1, 5, JOY_UP))
		{
			setRollers(127);
		}
		else if (joystickGetDigital(1, 5, JOY_DOWN))
			setRollers(-127);
		else if (joystickGetDigital(1, 7, JOY_UP))
			setRollers(60);
		else
			setRollers(0);

		// Press buttons to set arm position
		if (joystickGetDigital(1, 6, JOY_DOWN))
			moveArmsZeroAsync();
		else if (joystickGetDigital(1, 6, JOY_UP))
			moveArmsLowAsync();
		else if (joystickGetDigital(1, 8, JOY_UP))
			moveArmsMedAsync();
		else if (joystickGetDigital(1, 8, JOY_DOWN))
			stopAsyncArmController();

		// Toggle button for angling the tray
		if (joystickGetDigital(1, 7, JOY_DOWN))
		{
			if (isTrayVertical)
			{
				moveTrayAngledAsync();
				isTrayVertical = false;
			}
			else
			{
				moveTrayVerticalAsync();
				isTrayVertical = true;
			}
			delay(500);
		}
		else if (joystickGetDigital(1, 7, JOY_LEFT))
		{
			stopAsyncTrayController();
		}

		// Partner controls for overriding tray
		if (joystickGetDigital(2, 5, JOY_UP))
		{
			// Kill asyncTrayController
			stopAsyncTrayController();

			// Set tray power
			setTray(joystickGetAnalog(2, 3));
		}

		// Partner controls for overriding intake arms
		if (joystickGetDigital(2, 6, JOY_UP))
		{
			// Kill asyncArmController
			stopAsyncArmController();

			// Set arm power, depending if holding or not
			if (joystickGetDigital(2, 6, JOY_DOWN))
				setArms(20);
			else
				setArms(joystickGetAnalog(2, 2));
		}

		// Get and set power for drivetrains
		int leftPower = joystickGetAnalog(1, 3);
		int rightPower = joystickGetAnalog(1, 2);
		setDriveLinear(leftPower, rightPower);

		delay(20);
	}
}
