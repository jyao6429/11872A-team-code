#include "main.h"

namespace auton
{
    void corner2(int side)
    {
        // Line up with corner goal and intake 2 balls
        chassis::moveToTargetAsync(side * -45_in, -45_in, side * -135_deg, 1.0, 0.4, false);
        chassis::waitUntilStuck(3000);
        intake::setState(intake::NUMBER_IN, 2);
        chassis::moveToTargetAsync(side * -58_in, -58_in, side * -135_deg, 1.0, 0.4, false);
        chassis::waitUntilStuck(2000);

        // Score the two intaked balls
        scorer::setState(scorer::NUMBER_SCORE, 2);
        scorer::waitUntilStopped(3000);

        // Back away while outtaking 
        chassis::moveToTargetAsync(side * -45_in, -45_in, side * -135_deg, 1.0, 0.4, false);
        delay(400);
        intake::setState(intake::OUTTAKE);
        chassis::waitUntilStuck(2000);
        intake::setState(intake::OFF);
    }
    void matchStart(int side)
    {
        // Set starting position
        chassis::setOdomState(side * -15.7_in, -62.0_in, side * -90_deg);

        // Move to drop ball into goal and deploy
        chassis::moveToTargetAsync(side * -14_in, -57_in, side * -60_deg, 1.0, 0.6, false);
        chassis::waitUntilStuck(1000);
        intake::setState(intake::INTAKE);
        delay(750);
        intake::setState(intake::OFF);
    }

    void skills()
    {
        // Start infront of left ball, deploy and intake 2 balls
        chassis::setOdomState(-35.4_in, -61.1_in, 0_deg);
        intake::setState(intake::NUMBER_IN, 2);

        // Move to first 2 balls
        chassis::moveToTargetAsync(-35.4_in, -50_in, 0_deg, 1.0, 0.4, false);
        chassis::waitUntilStuck(1000);
        chassis::moveToTargetAsync(-58.6_in, -39_in, -65_deg, 1.0, 0.6, false);
        chassis::waitUntilStuck(2000);

        // Line up with corner goal and score
        indexer::moveVoltageSafe(10000);
        chassis::moveToTargetAsync(-53_in, -53_in, -135_deg, 1.0, 0.6, false);
        chassis::waitUntilStuck(2000);
        intake::setState(intake::NUMBER_IN, 2);
        chassis::moveToTargetAsync(-58_in, -58_in, -135_deg, 1.0, 0.4, false);
        chassis::waitUntilStuck(2000);
        scorer::setState(scorer::NUMBER_SCORE, 2);
        scorer::waitUntilStopped(5000);
        intake::waitUntilStopped(3000);

        // Back away while outtaking
        chassis::moveToTargetAsync(-50_in, -50_in, -135_deg, 1.0, 0.4, false);
        intake::setState(intake::OUTTAKE);
        chassis::waitUntilStuck(2000);
        intake::setState(intake::OFF);

        // Line up with center left ball and eject 2 blue balls
        chassis::moveToTargetAsync(-37_in, -40_in, 20_deg, 1.0, 0.6, false);
        chassis::waitUntilStuck(3000);
        intake::setState(intake::INTAKE);
        indexer::moveVoltageSafe(12000);
        scorer::setState(scorer::EJECT);

        // Intake center left ball
        chassis::moveToTargetAsync(-24_in, 0_in, 20_deg, 1.0, 0.4, false);
        delay(1500);
        intake::setState(intake::NUMBER_IN, 2);
        scorer::setState(scorer::OFF);
        chassis::waitUntilStuck(4000);

        // Line up with left middle goal and score
        chassis::moveToTargetAsync(-55.5_in, 0_in, -90_deg, 1.0, 0.8, false);
        chassis::waitUntilStuck(4000);
        scorer::setState(scorer::NUMBER_SCORE, 2);
        delay(1000);
        intake::setState(intake::NUMBER_IN, 1);
        scorer::waitUntilStopped(5000);
        intake::waitUntilStopped(2000);

        // Back away while outtaking
        chassis::moveToTargetAsync(-47_in, 0_in, -90_deg, 1.0, 0.8, false);
        indexer::moveVoltageSafe(10000);
        intake::setState(intake::OUTTAKE);
        chassis::waitUntilStuck(2000);



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

        // Intake ball from center
        chassis::moveToTargetAsync(-35.4_in, -20_in, 0_deg, 1.0, 0.4, false);
        chassis::waitUntilStuck(3000);
        intake::setState(intake::NUMBER_IN, 1);
        chassis::moveToTargetAsync(-35.4_in, -10_in, 0_deg, 1.0, 0.4, true);
        chassis::waitUntilStuck(2000);

        // Push ball into center goal
        chassis::moveToTargetAsync(-11_in, -11_in, 45_deg, 1.0, 0.6, false);
        chassis::waitUntilStuck(3000);


    }
    void rightHalf()
    {
        // Start normal match right of middle goal
        matchStart(-1);

        // Intake 2 and score 2 in back corner goal
        corner2(-1);

        // Possibly push out balls in middle goal?
        //intake::setState(intake::OUTTAKE);
        //chassis::moveToTargetAsync(58_in, -11_in, 35_deg, 1.0, 0.6, true);
        //chassis::waitUntilStuck(3000);
    }
}