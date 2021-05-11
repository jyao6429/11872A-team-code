#include "main.h"

namespace scorer
{
    okapi::Motor scorer(1);

    okapi::ControllerButton scorerPositiveButton(okapi::ControllerDigital::R1);
	okapi::ControllerButton scorerNegativeButton(okapi::ControllerDigital::R2);

    void opcontrol()
    {
        int scorerVolt = 0;

        if (scorerNegativeButton.isPressed())
			scorerVolt = -12000;
		else if (scorerPositiveButton.isPressed())
			scorerVolt = 12000;

        scorer.moveVoltage(scorerVolt);
    }
}