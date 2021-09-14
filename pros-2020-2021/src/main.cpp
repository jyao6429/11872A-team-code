#include "main.h"
#include "chassis.h"
#include "intake.h"
#include "odom.h"
#include "okapi/api/util/mathUtil.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "autoSelect/selection.h"
#include <cmath>
#include <memory>

Controller master(E_CONTROLLER_MASTER);

// Variables for textUpdate
int prevAuto = 1;
std::unique_ptr<Task> textUpdateHandler;

// Task for updating controller text for auton state
void textUpdate(void *ign)
{
	while (true)
	{
		int currentAuto = selector::auton;
		if (currentAuto != prevAuto)
		{
			char autoSelection[20];
			sprintf(autoSelection, "Auto: %d    ", currentAuto);
			master.set_text(0, 0, autoSelection);
			prevAuto = currentAuto;
		}
		delay(200);
	}
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	//odom::start(false);
	chassis::init();
	intake::init();
	scorer::init();

	selector::init();

	textUpdateHandler = std::make_unique<Task>(textUpdate, nullptr, TASK_PRIORITY_DEFAULT - 1);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
okapi::ControllerButton autoSelectUP(okapi::ControllerDigital::up);
okapi::ControllerButton autoSelectDown(okapi::ControllerDigital::down);

void disabled()
{
	chassis::setState(chassis::SKIP);
	chassis::stop();
	intake::setState(intake::OFF);
	intake::stop();
	scorer::setState(scorer::OFF);
	scorer::stop();

	/*
	while (true)
	{
		if (autoSelectUP.changedToPressed() && selector::auton < 4)
			selector::auton++;
		if (autoSelectDown.changedToPressed() && selector::auton > -4)
			selector::auton--;

		delay(50);
	}
	*/
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
	// Ensure all necessary subsystems are running
	chassis::init();
	intake::init();
	scorer::init();

	// Switch based on desired autonomous
	int selection = std::abs(selector::auton);
	switch (selection)
	{
		case 0:
			auton::skillsSafe();
			//auton::skills();
			//test::run();
			break;
		case 1:
			auton::leftHomeRow();
			break;
		case 2:
			auton::leftHalf();
			break;
		case 3:
			auton::rightHalf();
			break;
		case 4:
			disabled();
			break;
	}
	
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	chassis::setState(chassis::SKIP);
	chassis::stop();
	intake::setState(intake::OFF);
	intake::stop();
	scorer::setState(scorer::OFF);
	scorer::stop();

	int timer = millis();
	bool isLogging = false;

	while (true)
	{
		// Run each of the subsystem's opcontrol
		chassis::opcontrol();
		intake::opcontrol();
		indexer::opcontrol();
		scorer::opcontrol();

		if (millis() - timer > 100 && isLogging)
		{
			//odom::pose robotPose = odom::getPose();
			//printf("X: %3.3f\tY: %3.3f\tT: %3.3f\n", robotPose.x, robotPose.y, robotPose.theta * okapi::radianToDegree);
			//printf("L: %d\tR: %d\tB: %d\n", leftEncoder.get_value(), rightEncoder.get_value(), backEncoder.get_value());
			timer = millis();
		}
		delay(10);
	}
}
