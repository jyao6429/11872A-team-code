#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	printf("Initializing LCD\n");
	initLCD();
	printf("Initializing Arm\n");
	initArm();
	printf("Initializing Drive\n");
	initDrive();
	printf("Initializing Tray\n");
	initTray();
	printf("Done Initializing\n");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
	stopAPS();
	stopAsyncChassisController();
	stopTesting();
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
	bool isTesting = false;

	if (isConfirmed)
		isTesting = false;

	if (isTesting)
	{
		startTesting();
		return;
	}

	if (color == AUTO_COLOR_SKILLS)
	{
		return;
	}
	if (side == SIDE_SMALL)
  {
    switch (smallGoalAuto)
    {
      case SMALL_9PT:
				break;
      case SMALL_8PT:
				break;
      case SMALL_7PT:
				break;
    	case SMALL_6PT:
				break;
      case SMALL_5PT:
				break;
      case SMALL_1PT:
				break;
    }
  }
  else
  {
    switch (largeGoalAuto)
    {
      case LARGE_5PT:
				break;
      case LARGE_1PT:
				break;
    }
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
	Controller controller;

	moveArmsZeroAsync();
	moveTrayAngledAsync();

	// Controller buttons
	ControllerButton intakeButton(ControllerDigital::L1);
	ControllerButton outtakeButton(ControllerDigital::L2);
	ControllerButton intakeShiftButton(ControllerDigital::R2);
	ControllerButton armShiftButton(ControllerDigital::R1);
	ControllerButton trayToggleButton(ControllerDigital::down);
	ControllerButton backOutButton(ControllerDigital::left);
	ControllerButton trayOverrideButton(ControllerDigital::up);
	ControllerButton armOverrideButton(ControllerDigital::X);
	ControllerButton armResetButton(ControllerDigital::Y);

	// handles tray toggling
	bool isTrayVertical = false;

	while (true)
	{
		// Get power for drivetrain, match to cubic function for more precision in slow movements
		double leftPower = controller.getAnalog(ControllerAnalog::leftY);
		double rightPower = controller.getAnalog(ControllerAnalog::rightY);

		leftPower = pow(leftPower, 3);
		rightPower = pow(rightPower, 3);

		// Variable for roller power
		int rollerPower = 0;

		// Handle the arm shift
		if (armShiftButton.isPressed())
		{
			// Positions the arm based on button presses
			if (intakeShiftButton.changedToPressed())
				moveArmsZeroAsync();
			else if (outtakeButton.changedToPressed())
				moveArmsLowAsync();
			else if (intakeButton.changedToPressed())
				moveArmsMedAsync();
		}
		else
		{
			if (intakeButton.isPressed())
				rollerPower = 127;
			else if (outtakeButton.isPressed())
				rollerPower = -127;
		}

		// Handle intake shift
		if (intakeShiftButton.isPressed())
			rollerPower /= 2;

		// Handle tray toggle
		if (trayToggleButton.changedToPressed())
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
		}

		// Handle button for backing away from a stack
		if (backOutButton.isPressed())
		{
			leftPower = rightPower;
			rollerPower = rightPower * 127 * 1.7;
		}

		// Handle manual overrides
		if (trayOverrideButton.isPressed())
		{
			pauseAsyncTrayController();
			if ((getTrayPot() > TRAY_VERTICAL + 100 && rightPower > 0) || (getTrayPot() < TRAY_ANGLED && rightPower < 0))
				rightPower = 0;

			setTray(rightPower);
			rightPower = leftPower = 0;
		}
		if (armOverrideButton.isPressed())
		{
			nextArmTarget = -1;
			if ((getArmPosition() > ARM_MED + 500 && leftPower > 0) || (getArmPosition() < ARM_ZERO && leftPower < 0))
				leftPower = 0;

			setArms(leftPower);
			leftPower = rightPower = 0;
		}

		// Handle arm reset
		if (armResetButton.changedToPressed())
			resetArm();

		setDrive(leftPower, rightPower);
		if (rollerPower == 0)
			setRollersVel(0);
		else
			setRollers(rollerPower);

		pros::delay(10);
	}
}
