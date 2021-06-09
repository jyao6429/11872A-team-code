#include "main.h"

namespace auton
{
    void corner2(int side)
    {
        // Line up with corner goal and intake 2 balls
        intake::setState(intake::NUMBER_IN, 2);
        chassis::moveToTargetAsync((side * -45) * 1_in, -45_in, (side * -135) * 1_deg, 1.0, 0.4, false);
        chassis::waitUntilStuck(3000);
        chassis::moveToTargetAsync((side * -58) * 1_in, -58_in, (side * -135) * 1_deg, 1.0, 0.4, false);
        chassis::waitUntilStuck(2000);

        // Score the two intaked balls
        scorer::setState(scorer::NUMBER_SCORE, 2);
        scorer::waitUntilStopped(3000);

        // Back away while outtaking 
        chassis::moveToTargetAsync((side * -45) * 1_in, -45_in, (side * -135) * 1_deg, 1.0, 0.4, false);
        delay(400);
        intake::setState(intake::OUTTAKE);
        chassis::waitUntilStuck(2000);
        intake::setState(intake::OFF);
    }
    void matchStart(int side)
    {
        // Set starting position
        chassis::setOdomState((side * -15.7) * 1_in, -62.0_in, (side * -90) * 1_deg);

        // Move to drop ball into goal and deploy
        chassis::moveToTargetAsync((side * -14) * 1_in, -57_in, (side * -60) * 1_deg, 1.0, 0.6, false);
        chassis::waitUntilStuck(1000);
        intake::setState(intake::INTAKE);
        delay(750);
    }

    void skills()
    {
    }
    void skillsSafe()
    {
    }
    void leftHomeRow()
    {
        // Start normal match left of middle goal
        matchStart(1);

        // Intake 2 and score 2 in back corner goal
        corner2(1);

        // Travel across the field
        chassis::moveToTargetAsync(0_in, -35_in, -180_deg, 1.0, 0.4, false);
        chassis::waitUntilStuck(4000);

        // Intake 2 and score 2 in front corner goal
        corner2(-1);
    }
    void leftHalf()
    {
        // Start normal match left of middle goal
        matchStart(1);

        // Intake 2 and score 2 in back corner goal
        corner2(1);
    }
    void rightHalf()
    {
        // Start normal match right of middle goal
        matchStart(-1);

        // Intake 2 and score 2 in back corner goal
        corner2(1);
    }
}