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

	// Gather max linear velocity
	while (millis() - timer < 2500 && false)
	{
		setDrive(127, 127);
		logDataDouble("linearV", sqrt(pow(globalVel.x, 2) + pow(globalVel.y, 2)));
		delay(50);
	}

	// Gather max angular velocity
	while (millis() - timer < 2500 && true)
	{
		setDrive(-127, 127);
		logDataDouble("angularV", globalVel.angle);
		logDataDouble("angularVDeg", radToDeg(globalVel.angle));
		delay(50);
	}

	while (powerDiff <= 0 && false)
	{
		setDriveLinear(127 + powerDiff, 127);

		double angularV = globalVel.angle;

		// Debug
		logDataInt("powerDiff", powerDiff);
		logDataDouble("angularV80", angularV * 80);
		logDataDouble("angularV70", angularV * 70);
		logDataDouble("angularV60", angularV * 60);
		logDataDouble("angularV50", angularV * 50);
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

	//sweepTurnToTarget(12.0, 12.0, 90.0, 12, TURN_CW, 127, true, true);

	//moveToTargetSimple(0.0, 36.0, 0.0, 0.0, 127, 127, 1, 0, 0, 0, STOP_NONE, MTT_SIMPLE);
	//moveToTargetSimple(-36, 72, 0, 36, 127, 127, 0.5, 0, 50, 0, STOP_HARSH, MTT_CASCADING);

	moveToTargetSimpleAsync(0.0, 60.0, 0.0, 0.0, 127, 0, 1, 0, 0, 0, STOP_NONE, MTT_SIMPLE);
	print("~~~1~~~\n");
	delay(1500);
	print("~~~2~~~\n");
	turnToAngleNewAsync(-90, TURN_CCW, 0.9, 127, 0, false, true);
	print("~~~3~~~\n");
	waitUntilChassisMoveComplete();
	print("~~~4~~~\n");
	turnToTargetNewAsync(0.0, 0.0, TURN_CH, 0.7, 30, 12, 0, true, false);
	print("~~~5~~~\n");
	waitUntilChassisMoveComplete();
	print("~~~6~~~\n");
	moveToTargetSimpleAsync(0.0, 0.0, 0.0, globalPose.y, 127, 0, 0.5, 0, 20, 0, STOP_HARSH, MTT_CASCADING);
	print("~~~7~~~\n");
	waitUntilChassisMoveComplete();
	print("~~~8~~~\n");
}
void testAsyncNew()
{
	print("~~~1~~~\n");
	turnToAngleNewAsync(-90, TURN_CCW, 0.9, 127, 0, false, true);
	print("~~~2~~~\n");
	waitUntilChassisMoveComplete();
	print("~~~3~~~\n");
}
void testSonar()
{
	resetLeftAgainstWall(RESET_X_NEAR, 0, 100, 100, 500);

	while (false)
	{
		printf("SONAR: %3.3f\n", sonarReadFiltered(leftSonar, 0.0, 10000.0, 100, 500) + LEFT_SONAR_TO_CENTER);
		delay(10);
	}
}
void tuneWheelDiameter()
{
	moveToTargetSimpleAsync(0.0, 60.0, 0.0, 0.0, 127, 0, 0.5, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
	waitUntilChassisMoveComplete();
}
void tuneWheelbase()
{
	setDrive(40, 40);
	delay(1000);

	while (globalPose.angle < degToRad(720))
	{
		setDriveLinear(127, -127);
		printf("X: %3.3f   Y: %3.3f   A: %3.3f   XV: %3.3f   YV: %3.3f   AV: %3.3f\n", globalPose.x, globalPose.y, radToDeg(globalPose.angle), globalVel.x, globalVel.y, radToDeg(globalVel.angle));
		delay(50);
	}
	setDrive(0, 0);

	turnToAngleNewAsync(0.0, TURN_CCW, 0.7, 20, 10, true, true);
	waitUntilChassisMoveComplete();

	setDriveLinear(-30, -30);
	delay(4000);
	stopDrive();
}
void testTray()
{
	print("~~~1~~~\n");
	moveTrayVerticalAsync();
	print("~~~2~~~\n");
	waitUntilTrayMoveComplete();
	delay(2000);
	print("~~~3~~~\n");
	moveTrayAngledAsync();
	print("~~~4~~~\n");
	waitUntilTrayMoveComplete();
}
void testArms()
{
	moveTrayAngledAsync();
	print("~~~1~~~\n");
	moveArmsLowAsync();
	print("~~~2~~~\n");
	waitUntilArmMoveComplete();

	delay(2000);
	print("~~~3~~~\n");
	moveArmsMedAsync();
	print("~~~4~~~\n");
	waitUntilArmMoveComplete();

	delay(2000);
	print("~~~5~~~\n");
	moveArmsZeroAsync();
	waitUntilArmMoveComplete();
	print("~~~6~~~\n");
	waitUntilTrayMoveComplete();
}

/***************************************************************
 *                   Tasks to handle tests                     *
 ***************************************************************/
TaskHandle testHandler;
bool isTesting = false;

void testTask(void *ignore)
{
	// Reset all positions
	resetPositionFull(&globalPose, 0.0, 0.0, 0.0, true);
	resetVelocity(&globalVel, globalPose);

	// Call method for the test
	//gatherVelocityData();
	//testNewMotionAlgorithms();
	//testAsyncNew();
	//testTray();
	//testArms();
	//deploy();
	//tuneWheelDiameter();
	tuneWheelbase();
	//autonomous();


	// Stop everything
	stopAsyncChassisController();
	stopAsyncTrayController();
	stopAsyncArmController();
	stopRollers();

	// Debug if wanted
	while (true)
	{
		// Debug pots
		//printf("armPot: %d\tdialPot: %d\ttrayPot: %d\n", analogRead(PORT_armPot), analogRead(PORT_dialPot), analogRead(PORT_trayPot));
		printf("X: %3.3f   Y: %3.3f   A: %3.3f   XV: %3.3f   YV: %3.3f   AV: %3.3f\n", globalPose.x, globalPose.y, radToDeg(globalPose.angle), globalVel.x, globalVel.y, radToDeg(globalVel.angle));
		delay(50);
	}
	printf("Done Testing\n");
}
void startTesting()
{
	// Don't run the test if it is already running
	unsigned int testState = taskGetState(testHandler);
  if (testHandler != NULL && (testState != TASK_DEAD))
		return;

	print("Starting test\n");
	// Start the task
	testHandler = taskCreate(testTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT + 1);
}
void stopTesting()
{
	// Stop task if needed
  unsigned int testState = taskGetState(testHandler);
  if (testHandler != NULL && (testState != TASK_DEAD))
    taskDelete(testHandler);

	// Stop all motors and controllers
	stopAsyncChassisController();
	stopAsyncArmController();
	stopAsyncTrayController();
	stopRollers();

	print("Stopped testing\n");
}
