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
	printf("Starting operatorControl\n");

	while (true)
	{
		if (digitalRead(PORT_startTestButton) == LOW)
		{
			startTesting();
			delay(500);
		}
		if (digitalRead(PORT_stopTestButton) == LOW)
		{
			stopTesting();
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

		delay(20);
	}
}
