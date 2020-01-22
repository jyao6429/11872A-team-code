/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

void operatorControl()
{
	print("Starting operatorControl\n");

	startAsyncArmController();

	// Initialize toggle variables
	isTesting = false;
	bool isSlowDrive = false;

	// Set values for joystick connection
	isMainConnected = isJoystickConnected(1);
	isPartnerConnected = isJoystickConnected(2);

	while (true)
	{

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

		// Get power for drivetrains, match to cubic function for more precision in slow movements
		int leftPower = joystickGetAnalog(1, 3);
		int rightPower = joystickGetAnalog(1, 2);

		leftPower = pow((double) leftPower / 127.0, 3) * 127;
		rightPower = pow((double) rightPower / 127.0, 3) * 127;

		// Create variables for roller power
		int rollerPower = 0;

		// Handle the rollers if not controlling arms
		if (!joystickGetDigital(1, 6, JOY_UP))
		{
			if (joystickGetDigital(1, 5, JOY_UP))
				rollerPower = 127;
			else if (joystickGetDigital(1, 5, JOY_DOWN))
				rollerPower = -127;
		}

		// Shift for rollers (slows down by half)
		if (joystickGetDigital(1, 6, JOY_DOWN))
			rollerPower /= 2;

		if (isPartnerConnected)
		{
			// Partner controls for overriding tray
			if (joystickGetDigital(2, 5, JOY_UP))
			{
				// Set tray power
				setTray(joystickGetAnalog(2, 3));
			}
			// Partner controls for overriding intake arms
			if (joystickGetDigital(2, 6, JOY_UP))
			{
				// Set arm power, depending if holding or not
				if (joystickGetDigital(2, 6, JOY_DOWN))
					setArms(20);
				else
					setArms(joystickGetAnalog(2, 2));
			}
		}

		// Handle if slowing down drivetrain to prevent tipping
		if (joystickGetDigital(1, 8, JOY_RIGHT))
		{
			isSlowDrive = !isSlowDrive;
			delay(250);
		}
		if (isSlowDrive)
		{
			leftPower /= 2;
			rightPower /= 2;
		}

		// Hold button to back up slowly and outtake
		if (joystickGetDigital(1, 7, JOY_LEFT))
		{
			leftPower = -40;
			rightPower = -40;
		}

		if (joystickGetDigital(1, 8, JOY_LEFT))
			startAsyncArmController();

		// Manual overrides for tray and arms
		if (joystickGetDigital(1, 7, JOY_UP))
		{
			if ((getTrayPot() > TRAY_VERTICAL + 100 && rightPower > 0) || (getTrayPot() < TRAY_ANGLED && rightPower < 0))
			{
				rightPower = 0;
			}
			rightPower = (rightPower > 30 && getTrayPot() > TRAY_VERTICAL - 400) ? 50 : rightPower;

			setTray(rightPower);
			rightPower = 0;
		}
		if (joystickGetDigital(1, 8, JOY_UP))
		{
			setArms(leftPower);
			leftPower = 0;
		}

		setDriveLinear(leftPower, rightPower);
		setRollers(rollerPower);

		delay(20);
	}
}
