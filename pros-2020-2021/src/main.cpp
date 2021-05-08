#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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
void autonomous() {}

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
	Controller master;
	std::shared_ptr<ChassisController> chassis = ChassisControllerBuilder()
												.withMotors(15, -16, -20, 11)
												.withDimensions(AbstractMotor::gearset::green, {{3.25_in, 17_in}, imev5GreenTPR})
												.build();
	std::shared_ptr<XDriveModel> drive = std::dynamic_pointer_cast<XDriveModel>(chassis->getModel());

	Motor indexerMotor(-5);
	Motor scorerMotor(1);
	Motor leftIntake(6);
	Motor rightIntake(-10);

	ControllerButton intakePositiveButton(ControllerDigital::L1);
	ControllerButton intakeNegativeButton(ControllerDigital::L2);
	ControllerButton scorerPositiveButton(ControllerDigital::R1);
	ControllerButton scorerNegativeButton(ControllerDigital::R2);


	while (true)
	{
		drive->xArcade(master.getAnalog(ControllerAnalog::leftX), master.getAnalog(ControllerAnalog::leftY), master.getAnalog(ControllerAnalog::rightX), 0);

		int intakeVolt = 0;
		int indexerVolt = 0;
		int scorerVolt = 0;

		if (intakePositiveButton.isPressed())
		{
			intakeVolt = 12000;
			indexerVolt = 12000;
		}
		else if (intakeNegativeButton.isPressed())
		{
			intakeVolt = -12000;
			indexerVolt = -12000;
		}
		if (scorerPositiveButton.isPressed())
		{
			scorerVolt = 12000;
			indexerVolt = 12000;
		}
		else if (scorerNegativeButton.isPressed())
		{
			scorerVolt = -12000;
		}

		leftIntake.moveVoltage(intakeVolt);
		rightIntake.moveVoltage(intakeVolt);
		indexerMotor.moveVoltage(indexerVolt);
		scorerMotor.moveVoltage(scorerVolt);

		pros::delay(10);
	}
}
