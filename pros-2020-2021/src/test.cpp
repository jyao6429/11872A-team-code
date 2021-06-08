#include "chassis.h"
#include "intake.h"
#include "main.h"
#include "okapi/api/util/mathUtil.hpp"
#include "pros/rtos.hpp"

namespace test
{
    // Enum for each test case
    enum TestScript
    {
        INTAKE_1,
        INTAKE_2,
        SCORER_1,
        SCORER_2,
        IN_SCORE_COMBINED_1_1,
        IN_SCORE_COMBINED_2_1,
        IN_SCORE_COMBINED_1_2,
        IN_SCORE_COMBINED_2_2,
        MTT_STRAIGHT,
        MTT_TURN,
        MTT_COMBINED_0,
        MTT_COMBINED_1,
        MTT_COMBINED_2,
        MTT_ASYNC_0,
        MTT_ASYNC_1,
        MTT_ASYNC_2,
        MTT_ASYNC_3,
        ODOM_DIAMETER_TUNE
    };
    // Current test case to run
    static constexpr TestScript currentTest = MTT_ASYNC_0;
    
    void run()
    {
        printf("Starting Test\n");
        switch (currentTest)
        {
            case INTAKE_1:
                intake::init();
                printf("Starting INTAKE_1\n");
                intake::setState(intake::NUMBER_IN, 1);
                printf("Waiting INTAKE_1\n");
                intake::waitUntilStopped();
                printf("Done INTAKE_1\n");
                break;
            case INTAKE_2:
                intake::init();
                printf("Starting INTAKE_2\n");
                intake::setState(intake::NUMBER_IN, 2);
                printf("Waiting INTAKE_2\n");
                intake::waitUntilStopped();
                printf("Done INTAKE_2\n");
                break;
            case SCORER_1:
                scorer::init();
                printf("Starting SCORER_1\n");
                scorer::setState(scorer::NUMBER_SCORE, 1);
                printf("Waiting SCORER_1\n");
                scorer::waitUntilStopped();
                printf("Done SCORER_1\n");
                break;
            case SCORER_2:
                scorer::init();
                printf("Starting SCORER_2\n");
                scorer::setState(scorer::NUMBER_SCORE, 2);
                printf("Waiting SCORER_2\n");
                scorer::waitUntilStopped();
                printf("Done SCORER_2\n");
                break;
            case IN_SCORE_COMBINED_1_1:
                intake::init();
                scorer::init();
                printf("Starting IN_SCORE_COMBINED_1_1\n");
                intake::setState(intake::NUMBER_IN, 1);
                scorer::setState(scorer::NUMBER_SCORE, 1);
                printf("Waiting IN_SCORE_COMBINED_1_1\n");
                intake::waitUntilStopped();
                scorer::waitUntilStopped();
                printf("Done IN_SCORE_COMBINED_1_1\n");
                break;
            case IN_SCORE_COMBINED_2_1:
                intake::init();
                scorer::init();
                printf("Starting IN_SCORE_COMBINED_2_1\n");
                intake::setState(intake::NUMBER_IN, 2);
                scorer::setState(scorer::NUMBER_SCORE, 1);
                printf("Waiting IN_SCORE_COMBINED_2_1\n");
                intake::waitUntilStopped();
                scorer::waitUntilStopped();
                printf("Done IN_SCORE_COMBINED_2_1\n");
                break;
            case IN_SCORE_COMBINED_1_2:
                intake::init();
                scorer::init();
                printf("Starting IN_SCORE_COMBINED_1_2\n");
                intake::setState(intake::NUMBER_IN, 1);
                scorer::setState(scorer::NUMBER_SCORE, 2);
                printf("Waiting IN_SCORE_COMBINED_1_2\n");
                intake::waitUntilStopped();
                scorer::waitUntilStopped();
                printf("Done IN_SCORE_COMBINED_1_2\n");
                break;
            case IN_SCORE_COMBINED_2_2:
                intake::init();
                scorer::init();
                printf("Starting IN_SCORE_COMBINED_2_2\n");
                intake::setState(intake::NUMBER_IN, 2);
                scorer::setState(scorer::NUMBER_SCORE, 2);
                printf("Waiting IN_SCORE_COMBINED_2_2\n");
                intake::waitUntilStopped();
                scorer::waitUntilStopped();
                printf("Done IN_SCORE_COMBINED_2_2\n");
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
                chassis::moveToTarget(0_in, 48_in, 90_deg, 1.0, 0.3, true);
                printf("Done MTT_COMBINED_0\n");
                break;
            case MTT_COMBINED_1:
                printf("Starting MTT_COMBINED_1\n");
                chassis::resetOdom();
                chassis::moveToTarget(48_in, 48_in, 180_deg, 1.0, 0.3, true);
                printf("Done MTT_COMBINED_1\n");
                break;
            case MTT_COMBINED_2:
                printf("Starting MTT_COMBINED_2\n");
                chassis::resetOdom();
                chassis::moveToTarget(0_in, 48_in, -90_deg, 1.0, 0.3, false);
                chassis::moveToTarget(48_in, 48_in, 180_deg, 1.0, 0.4, true);
                printf("Done MTT_COMBINED_2\n");
                break;
            case MTT_ASYNC_0:
                printf("Starting MTT_ASYNC_0\n");
                chassis::resetOdom();
                chassis::init();
                chassis::moveToTargetAsync(24_in, 48_in, 180_deg, 1.0, 0.4, true);
                chassis::waitUntilSettled();
                chassis::moveToTargetAsync(0_in, 0_in, 0_deg, 1.0, 0.4, true);
                chassis::waitUntilSettled();
                break;
            case MTT_ASYNC_1:
                printf("Starting MTT_ASYNC_1\n");
                chassis::resetOdom();
                chassis::init();
                chassis::moveToTargetAsync(0_in, 48_in, -90_deg, 1.0, 0.3, false);
                chassis::waitUntilSettled();
                chassis::moveToTargetAsync(48_in, 48_in, 180_deg, 1.0, 0.4, true);
                chassis::waitUntilSettled();
                break;
            case MTT_ASYNC_2:
                printf("Starting MTT_ASYNC_2\n");
                chassis::resetOdom();
                chassis::init();
                chassis::moveToTargetAsync(0_in, 48_in, -90_deg, 1.0, 0.3, false);
                chassis::waitUntilSettled(500);
                chassis::moveToTargetAsync(48_in, 48_in, 180_deg, 1.0, 0.4, true);
                chassis::waitUntilSettled();
                break;
            case MTT_ASYNC_3:
                printf("Starting MTT_ASYNC_3\n");
                chassis::resetOdom();
                chassis::init();
                chassis::moveToTargetAsync(0_in, 48_in, 0_deg, 1.0, 0.3, true);
                printf("Finished MTT_ASYNC_3 code: %d\n", chassis::waitUntilStuck(100000));
                break;
            case ODOM_DIAMETER_TUNE:
                printf("Starting ODOM_DIAMETER_TUNE\n");
                //chassis::moveVector(-okapi::pi / 2, 0, 0.3);
                delay(500);
                chassis::resetOdom();
                chassis::init();
                chassis::moveToTargetAsync(0_in, 120_in, 0_deg, 0.7, 0.5, true);
                printf("Waiting ODOM_DIAMETER_TUNE\n");
                chassis::waitUntilSettled();
                printf("Done waiting ODOM_DIAMETER_TUNE\n");
                while (true)
                {
                    chassis::opcontrol();
                    delay(50);
                }
                break;
        }
        printf("Done Test\n");
    }
}