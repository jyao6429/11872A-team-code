#include "intake.h"
#include "main.h"

namespace test
{
    enum TestScript
    {
        INTAKE_1,
        INTAKE_2
    };
    static constexpr TestScript currentTest = INTAKE_1;
    
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
        }
        printf("Done Test\n");
    }
}