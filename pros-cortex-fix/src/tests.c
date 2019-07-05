#include "main.h"

void testChassis(void *ignore)
{
	resetPositionFull(&globalPose, 0.0, 0.0, 0.0, false);
	// Test turning PID
	turnToAngle(90.0, 100, true, true);
	turnToAngle(0.0, 100, true, false);

	printf("Done Testing\n");
	stopMotors();
}
void gatherVelocityData(void *ignore)
{
	resetPositionFull(&globalPose, 0.0, 0.0, 0.0, true);
	resetVelocity(&globalVel, globalPose);

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

	//moveToTargetSimple(0.0, 36.0, 0.0, 0.0, 127, 127, 1, 0, 0, 0, STOP_NONE, MTT_SIMPLE);
	//moveToTargetSimple(-36, 72, 0, 36, 127, 127, 0.5, 0, 50, 0, STOP_HARSH, MTT_CASCADING);

	while (false)
	{
		printf("X: %3.3f   Y: %3.3f   A: %3.3f\n", globalPose.x, globalPose.y, radToDeg(globalPose.angle));
		delay(50);
	}
	printf("Done Testing\n");
}
