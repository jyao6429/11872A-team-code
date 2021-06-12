#include "chassis.h"
#include "indexer.h"
#include "intake.h"
#include "main.h"
#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/odometry/odomState.hpp"
#include "pros/rtos.hpp"
#include "scorer.h"

namespace auton
{
    void corner2(int side)
    {
        // Line up with corner goal and intake 2 balls
        chassis::moveToTargetAsync(side * -45_in, -45_in, side * -135_deg, 1.0, 0.4, false);
        chassis::waitUntilStuck(3000);
        intake::setState(intake::NUMBER_IN, 1);
        chassis::moveToTargetAsync(side * -58_in, -58_in, side * -135_deg, 1.0, 0.4, false);
        chassis::waitUntilStuck(2000);

        // Score the two intaked balls
        scorer::setState(scorer::NUMBER_SCORE, 1);
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
    void halfSafeSkills()
    {
        // Line up with corner goal and score
        chassis::moveToTargetAsync(-53_in, -53_in, -135_deg, 1.0, 0.6, false);
        chassis::waitUntilStuck(2000);
        chassis::moveToTargetAsync(-58_in, -58_in, -135_deg, 1.0, 0.4, false);
        chassis::waitUntilStuck(2000);
        scorer::setState(scorer::NUMBER_SCORE, 1);
        scorer::waitUntilStopped(5000);

        // Back away while intaking
        chassis::moveToTargetAsync(-50_in, -50_in, -135_deg, 1.0, 0.4, false);
        intake::setState(intake::INTAKE);
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
        scorer::waitUntilStopped(5000);

        // Back away while intaking
        chassis::moveToTargetAsync(-47_in, 0_in, -90_deg, 1.0, 0.8, false);
        intake::setState(intake::INTAKE);
        chassis::waitUntilStuck(2000);
        intake::setState(intake::OFF);

        // Intake one ball and line up with corner goal
        intake::setState(intake::NUMBER_IN, 1);
        chassis::moveToTargetAsync(-36_in, 48_in, 20_deg, 1.0, 0.8, false);
        chassis::waitUntilStuck(3000);
        chassis::moveToTargetAsync(-53_in, 53_in, -45_deg, 1.0, 0.6, false);
        chassis::waitUntilStuck(2000);
        chassis::moveToTargetAsync(-58_in, 58_in, -45_deg, 1.0, 0.4, false);
        chassis::waitUntilStuck(2000);
        scorer::setState(scorer::NUMBER_SCORE, 1);
        scorer::waitUntilStopped(5000);

        // Back away while intaking
        chassis::moveToTargetAsync(-50_in, 50_in, -45_deg, 1.0, 0.4, false);
        intake::setState(intake::INTAKE);
        chassis::waitUntilStuck(2000);

        // Line up with center top ball and eject 1 blue ball
        chassis::moveToTargetAsync(-40_in, 37_in, 110_deg, 1.0, 0.6, false);
        chassis::waitUntilStuck(3000);
        intake::setState(intake::INTAKE);
        indexer::moveVoltageSafe(12000);
        scorer::setState(scorer::EJECT);

        // Intake center top ball
        chassis::moveToTargetAsync(0_in, 24_in, 110_deg, 1.0, 0.4, false);
        delay(1500);
        intake::setState(intake::NUMBER_IN, 1);
        scorer::setState(scorer::OFF);
        chassis::waitUntilStuck(4000);
    }
    void skillsSafe()
    {
        // Start regular match
        matchStart(1);
        intake::setState(intake::NUMBER_IN, 1);

        // Move to first ball
        chassis::moveToTargetAsync(-35.4_in, -48_in, -60_deg, 1.0, 0.8, false);
        chassis::waitUntilStuck(1000);

        // Half of skills
        halfSafeSkills();

        // Line up with top middle goal and score
        chassis::moveToTargetAsync(0_in, 55.5_in, 0_deg, 1.0, 0.8, false);
        chassis::waitUntilStuck(4000);
        scorer::setState(scorer::NUMBER_SCORE, 1);
        scorer::waitUntilStopped(5000);

        // Back away while intaking
        chassis::moveToTargetAsync(0_in, 48_in, 0_deg, 1.0, 0.8, false);
        intake::setState(intake::INTAKE);
        chassis::waitUntilStuck(2000);
        intake::setState(intake::OFF);

        // Turn to right corner ball and eject blue ball
        chassis::moveToTargetAsync(0_in, 48_in, 90_deg, 1.0, 1.0, false);
        chassis::waitUntilStuck(2000);
        intake::setState(intake::INTAKE);
        indexer::moveVoltageSafe(12000);
        scorer::setState(scorer::EJECT);

        // Intake right corner ball
        chassis::moveToTargetAsync(36_in, 48_in, 90_deg, 1.0, 0.4, true);
        delay(1500);
        intake::setState(intake::NUMBER_IN, 1);
        scorer::setState(scorer::OFF);
        chassis::waitUntilStuck(2000);

        delay(500);
        okapi::OdomState currentState = chassis::getOdomState();
        chassis::setOdomState(-currentState.x, -currentState.y, currentState.theta - 180_deg);

        halfSafeSkills();

        // Line up with center goal and score
        chassis::moveToTargetAsync(0_in, 8_in, -180_deg, 1.0, 0.8, false);
        intake::setState(intake::INTAKE);
        indexer::moveVoltageSafe(9000);
        chassis::waitUntilStuck(4000);
        chassis::setState(chassis::ROCK);
        delay(500);
        scorer::setState(scorer::NUMBER_SCORE, 1);
        scorer::waitUntilStopped(5000);

        // back up while outtaking
        intake::setState(intake::OUTTAKE);
        chassis::moveToTargetAsync(0_in, 20_in, -180_deg, 1.0, 0.8, false);
        chassis::waitUntilStuck(4000);
        intake::setState(intake::OFF);
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
        chassis::moveToTargetAsync(-36.0_in, -30_in, 0_deg, 1.0, 0.5, false);
        chassis::waitUntilStuck(3000);
        intake::setState(intake::NUMBER_IN, 1);
        chassis::moveToTargetAsync(-36.0_in, -12_in, 0_deg, 1.0, 0.4, true);
        chassis::waitUntilStuck(2000);

        // Push ball into center goal
        chassis::moveToTargetAsync(-11_in, -12_in, 45_deg, 1.0, 0.6, false);
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