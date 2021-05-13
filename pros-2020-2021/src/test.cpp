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
        MTT_COMBINED_0
    };
    static constexpr TestScript currentTest = MTT_STRAIGHT;
    
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
                printf("Starting INTAKE_1n");
                intake::setState(intake::NUMBER_IN, 1);
                printf("Waiting INTAKE_1\n");
                intake::waitUntilStopped();
                printf("Done INTAKE_1\n");
                break;
            case MTT_STRAIGHT:
                chassis::moveToTarget(0_in, 24_in, 0_deg, 1.0, 0.4, true);
                break;
            case MTT_TURN:
                chassis::moveToTarget(0_in, 0_in, 90_deg, 0.4, 1.0, true);
                break;
            case MTT_COMBINED_0:
                chassis::moveToTarget(0_in, 24_in, 90_deg, 1.0, 0.6, true);
                break;
        }
        printf("Done Test\n");
    }
}