#include "intake.h"
#include "main.h"

namespace test
{
    enum TestScript
    {
        INTAKE_1,
        INTAKE_2,
        MTT_STRAIGHT,
        MTT_TURN,
        MTT_COMBINED_0,
        MTT_COMBINED_1
    };
    static constexpr TestScript currentTest = MTT_TURN;
    
    void run()
    {
        printf("Starting Test\n");
        switch (currentTest)
        {
            case INTAKE_2:
                intake::init();
                printf("Starting INTAKE_2\n");
                intake::setState(intake::NUMBER_IN, 2);
                printf("Waiting INTAKE_2\n");
                intake::waitUntilStopped();
                printf("Done INTAKE_2\n");
                break;
            case INTAKE_1:
                intake::init();
                printf("Starting INTAKE_1\n");
                intake::setState(intake::NUMBER_IN, 1);
                printf("Waiting INTAKE_1\n");
                intake::waitUntilStopped();
                printf("Done INTAKE_1\n");
                break;
            case MTT_STRAIGHT:
                printf("Starting MTT_STRAIGHT\n");
                chassis::resetOdom();
                chassis::moveToTarget(0_in, 48_in, 0_deg, 1.0, 0.4, true);
                printf("Done MTT_STRAIGHT\n");
                break;
            case MTT_TURN:
                printf("Starting MTT_TURN\n");
                chassis::resetOdom();
                chassis::moveToTarget(0_in, 0_in, 90_deg, 1.0, 1.0, true);
                printf("Done MTT_TURN\n");
                break;
            case MTT_COMBINED_0:
                printf("Starting MTT_COMBINED_0\n");
                chassis::resetOdom();
                chassis::moveToTarget(0_in, 48_in, 90_deg, 1.0, 0.6, true);
                printf("Done MTT_COMBINED_0\n");
                break;
            case MTT_COMBINED_1:
                printf("Starting MTT_COMBINED_1\n");
                chassis::resetOdom();
                chassis::moveToTarget(48_in, 48_in, 180_deg, 1.0, 0.6, true);
                printf("Done MTT_COMBINED_1\n");
                break;
        }
        printf("Done Test\n");
    }
}