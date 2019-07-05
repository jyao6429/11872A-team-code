#include "main.h"

void testChassis()
{
	// Test turning PID
	turnToAngle(90.0, 100, true, true);
	turnToAngle(0.0, 100, true, false);
}
void gatherVelocityData()
{
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
}
void testNewMotionAlgorithms()
{
	//turnToTargetNew(-12, 0, TURN_CCW, 0.5, 25, 12, 0.0, true, true);

	sweepTurnToTarget(12.0, 12.0, 90.0, 12, TURN_CW, 127, true, true);

	//moveToTargetSimple(0.0, 36.0, 0.0, 0.0, 127, 127, 1, 0, 0, 0, STOP_NONE, MTT_SIMPLE);
	//moveToTargetSimple(-36, 72, 0, 36, 127, 127, 0.5, 0, 50, 0, STOP_HARSH, MTT_CASCADING);
}

TaskHandle testHandler;

void testTask(void *ignore)
{
	resetPositionFull(&globalPose, 0.0, 0.0, 0.0, true);
	resetVelocity(&globalVel, globalPose);

	testNewMotionAlgorithms();

	// Debug if wanted
	while (false)
	{
		printf("X: %3.3f   Y: %3.3f   A: %3.3f   XV: %3.3f   YV: %3.3f   AV: %3.3f\n", globalPose.x, globalPose.y, radToDeg(globalPose.angle), globalVel.x, globalVel.y, radToDeg(globalVel.angle));
		delay(50);
	}
}
void startTesting()
{
	// Don't run the test if it is already running
	unsigned int testState = taskGetState(testHandler);
  if (testHandler != NULL && (testState == TASK_RUNNING || testState == TASK_SLEEPING || testState == TASK_SUSPENDED))
		return;

	print("Starting test\n");
	// Start the task
	testHandler = taskCreate(testTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT + 1);
	stopMotors();
	printf("Done Testing\n");
}
void stopTesting()
{
	// Stop task if needed
  unsigned int testState = taskGetState(testHandler);
  if (testHandler != NULL && (testState == TASK_RUNNING || testState == TASK_SLEEPING || testState == TASK_SUSPENDED))
    taskDelete(testHandler);

	stopMotors();
	print("Stopped testing\n");
}
