#include "main.h"

namespace intake
{
    okapi::Motor left(6), right(-10);

    okapi::ControllerButton intakePositiveButton(okapi::ControllerDigital::L1);
	okapi::ControllerButton intakeNegativeButton(okapi::ControllerDigital::L2);

    void opcontrol()
    {
        int intakeVolt = 0;

        if (intakePositiveButton.isPressed())
			intakeVolt = 12000;
		else if (intakeNegativeButton.isPressed())
			intakeVolt = -12000;

        left.moveVoltage(intakeVolt);
		right.moveVoltage(intakeVolt);
    }
}