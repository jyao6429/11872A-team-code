#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	// Initialize tracking wheel encoders
	leftEncoder(PORT_leftEncoder, PORT_leftEncoder + 1, true);
	rightEncoder(PORT_rightEncoder, PORT_rightEncoder + 1, true);
	backEncoder(PORT_backEncoder, PORT_backEncoder + 1, true);

	// Initialize trayPot
	trayPot(PORT_trayPot);

	// Initialize non-chassis motors
	trayMotor(PORT_tray);
	trayMotor.setGearing(AbstractMotor::gearset::red);

	armMotor(PORT_arm);
	armMotor.setGearing(AbstractMotor::gearset::red);
	armMotor.setBrakeMode(AbstractMotor::brakeMode::hold);

	leftIntakeMotor(PORT_leftRoller);
	leftIntakeMotor.setGearing(AbstractMotor::gearset::green);
	rightIntakeMotor(PORT_rightRoller);
	rightIntakeMotor.setGearing(AbstractMotor::gearset::green);

	chassis = ChassisControllerBuilder()
						.withMotors({PORT_leftMotor0, PORT_leftMotor1}, {-PORT_rightMotor0, -PORT_rightMotor1})
						.withDimensions(AbstractMotor::gearset::green, {{4_in, 15.5_in}, imev5GreenTPR})
						.build();
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
void autonomous()
{
	// Choose the correct autonomous
  switch (chosenAuto)
  {
    case AUTO_BLUE_SMALL:
      autoBlueSmallSuperSafe();
      break;
    case AUTO_BLUE_LARGE:
      autoBlueLargeSuperSafe();
      break;
    case AUTO_RED_SMALL:
      autoRedSmallSuperSafe();
      break;
    case AUTO_RED_LARGE:
      autoRedLargeSuperSafe();
      break;
    case AUTO_SKILLS:
      autoSkillsSuperSafe();
      break;
    case AUTO_NONE:
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
	while (true)
	{
		double leftPower = controller.getAnalog(ControllerAnalog::leftY);
		double rightPower = controller.getAnalog(ControllerAnalog::rightY);

		leftPower = pow(leftPower, 3);
		rightPower = pow(rightPower, 3);




		setDrive(leftPower, rightPower);

		pros::delay(10);
	}
}
