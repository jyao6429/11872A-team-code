#include "main.h"

namespace indexer
{
    okapi::Motor indexer(-5);

    okapi::ControllerButton intakePositiveButton(okapi::ControllerDigital::L1);
	okapi::ControllerButton intakeNegativeButton(okapi::ControllerDigital::L2);
	okapi::ControllerButton scorerPositiveButton(okapi::ControllerDigital::R1);

    Mutex indexerMutex;

    void moveVoltageSafe(int indexerVolt)
    {
        indexerMutex.take(10);
        moveVoltage(indexerVolt);
        indexerMutex.give();
    }

    void moveVoltage(int indexerVolt)
    {
        indexer.moveVoltage(indexerVolt);
    }
    void opcontrol()
    {
        int indexerVolt = 0;

        if (intakePositiveButton.isPressed() || scorerPositiveButton.isPressed())
			indexerVolt = 12000;
		else if (intakeNegativeButton.isPressed())
			indexerVolt = -12000;

        moveVoltage(indexerVolt);
    }
}