#include "main.h"

void testNewMotionAlgorithms()
{
	//turnToTargetNew(-12, 0, TURN_CCW, 0.5, 25, 12, 0.0, true, true);

	//sweepTurnToTarget(12.0, 12.0, 90.0, 12, TURN_CW, 127, true, true);

	//moveToTargetSimple(0.0, 36.0, 0.0, 0.0, 127, 127, 1, 0, 0, 0, STOP_NONE, MTT_SIMPLE);
	//moveToTargetSimple(-36, 72, 0, 36, 127, 127, 0.5, 0, 50, 0, STOP_HARSH, MTT_CASCADING);

	moveToTargetSimpleAsync(0.0, 60.0, 0.0, 0.0, 127, 0, 1, 0, 0, 0, STOP_NONE, MTT_SIMPLE);
	printf("~~~1~~~\n");
	pros::delay(1500);
	printf("~~~2~~~\n");
	turnToAngleNewAsync(-90, TURN_CCW, 0.9, 127, 0, false, true);
	printf("~~~3~~~\n");
	waitUntilChassisMoveComplete();
	printf("~~~4~~~\n");
	turnToTargetNewAsync(0.0, 0.0, TURN_CH, 0.7, 30, 12, 0, true, false);
	printf("~~~5~~~\n");
	waitUntilChassisMoveComplete();
	printf("~~~6~~~\n");
	moveToTargetSimpleAsync(0.0, 0.0, 0.0, globalPose.y, 127, 0, 0.5, 0, 20, 0, STOP_HARSH, MTT_CASCADING);
	printf("~~~7~~~\n");
	waitUntilChassisMoveComplete();
	printf("~~~8~~~\n");
}
void testAsyncNew()
{
	printf("~~~1~~~\n");
	turnToAngleNewAsync(-90, TURN_CCW, 0.9, 127, 0, false, true);
	printf("~~~2~~~\n");
	waitUntilChassisMoveComplete();
	printf("~~~3~~~\n");
}
void tuneWheelDiameter()
{
	moveToTargetSimpleAsync(0.0, 60.0, 0.0, 0.0, 127, 0, 0.5, 0, 0, 0, STOP_NONE, MTT_PROPORTIONAL);
	waitUntilChassisMoveComplete();
}
void tuneWheelbase()
{
	setDrive(40, 40);
	pros::delay(1000);

	while (globalPose.angle < degToRad(720))
	{
		setDrive(127, -127);
		printf("X: %3.3f   Y: %3.3f   A: %3.3f   XV: %3.3f   YV: %3.3f   AV: %3.3f\n", globalPose.x, globalPose.y, radToDeg(globalPose.angle), globalVel.x, globalVel.y, radToDeg(globalVel.angle));
		pros::delay(50);
	}
	setDrive(0, 0);

	turnToAngleNewAsync(0.0, TURN_CCW, 0.7, 20, 10, true, true);
	waitUntilChassisMoveComplete();

	setDrive(-30, -30);
	pros::delay(4000);
	stopDrive();
}
void testTray()
{
	printf("~~~1~~~\n");
	moveTrayVerticalAsync();
	printf("~~~2~~~\n");
	waitUntilTrayMoveComplete();
	pros::delay(2000);
	printf("~~~3~~~\n");
	moveTrayAngledAsync();
	printf("~~~4~~~\n");
	waitUntilTrayMoveComplete();
}
void testArms()
{
	moveTrayAngledAsync();
	printf("~~~1~~~\n");
	moveArmsLowAsync();
	printf("~~~2~~~\n");
	waitUntilArmMoveComplete();

	pros::delay(2000);
	printf("~~~3~~~\n");
	moveArmsMedAsync();
	printf("~~~4~~~\n");
	waitUntilArmMoveComplete();

	pros::delay(2000);
	printf("~~~5~~~\n");
	moveArmsZeroAsync();
	waitUntilArmMoveComplete();
	printf("~~~6~~~\n");
	waitUntilTrayMoveComplete();
}
void intakeTest()
{
	moveArmsZeroAsync();
	moveTrayAngledAsync();
	setRollers(127);
	pros::delay(100000);
}

/***************************************************************
 *                   Tasks to handle tests                     *
 ***************************************************************/
std::unique_ptr<pros::Task> testHandler;

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
	//tuneWheelbase();
	autonomous();
	//intakeTest();


	// Stop everything
	stopAsyncChassisController();
	stopAsyncTrayController();
	stopAsyncArmController();
	stopRollers();

	// Debug if wanted
	while (false)
	{
		// Debug pots
		//printf("armPot: %d\tdialPot: %d\ttrayPot: %d\n", analogRead(PORT_armPot), analogRead(PORT_dialPot), analogRead(PORT_trayPot));
		printf("X: %3.3f   Y: %3.3f   A: %3.3f   XV: %3.3f   YV: %3.3f   AV: %3.3f\n", globalPose.x, globalPose.y, radToDeg(globalPose.angle), globalVel.x, globalVel.y, radToDeg(globalVel.angle));
		pros::delay(50);
	}
	printf("Done Testing\n");
}
void startTesting()
{
	// Don't run the test if it is already running
	if (testHandler != NULL && (testHandler->get_state() != pros::E_TASK_STATE_DELETED) && (testHandler->get_state() != pros::E_TASK_STATE_INVALID))
		return;

	printf("Starting test\n");
	// Start the task
	testHandler = std::make_unique<pros::Task>(testTask, nullptr, TASK_PRIORITY_DEFAULT + 1);
}
void stopTesting()
{
	// Stop task if needed
	if (testHandler != NULL && (testHandler->get_state() != pros::E_TASK_STATE_DELETED) && (testHandler->get_state() != pros::E_TASK_STATE_INVALID))
    testHandler->remove();

	// Stop all motors and controllers
	stopAsyncChassisController();
	stopAsyncArmController();
	stopAsyncTrayController();
	stopRollers();

	printf("Stopped testing\n");
}
