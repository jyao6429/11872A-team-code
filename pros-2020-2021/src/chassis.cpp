#include "main.h"
#include "okapi/api/odometry/odomState.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>

namespace chassis
{
    // X-drive chassis motors
    okapi::Motor topLeft(15), topRight(-16), bottomRight(-20), bottomLeft(11);

    // Okapi chassis controller
    auto chassisController = okapi::ChassisControllerBuilder()
        .withMotors(topLeft, topRight, bottomRight, bottomLeft)
        .withDimensions(okapi::AbstractMotor::gearset::green, {{3.25_in, 17_in}, okapi::imev5GreenTPR})
        .withSensors(
            okapi::ADIEncoder{'A', 'B'},
            okapi::ADIEncoder{'C', 'D'},
            okapi::ADIEncoder{'E', 'F', true}
        )
        .withOdometry({{2.73228_in, 14.866_in, 14.866_in / 2, 2.73228_in}, okapi::quadEncoderTPR}, okapi::StateMode::CARTESIAN)
        .buildOdometry();
    
    std::shared_ptr<okapi::XDriveModel> drive = std::dynamic_pointer_cast<okapi::XDriveModel>(chassisController->getModel());

    // MTT variables
    bool isSettled = true;
    bool initialized = false;
    ChassisState state = OFF;
    MTTContainer mttContainer;
    std::unique_ptr<Mutex> chassisMutex;
    std::unique_ptr<Task> chassisTaskhandler;


    // Override buttons and variables during opcontrol
    okapi::ControllerButton intakePositiveButton(okapi::ControllerDigital::L1);
	okapi::ControllerButton intakeNegativeButton(okapi::ControllerDigital::L2);
    okapi::ControllerButton strafeHorizontalButton(okapi::ControllerDigital::A);
	okapi::ControllerButton strafeVerticalButton(okapi::ControllerDigital::B);
	okapi::ControllerButton resetOdomButton(okapi::ControllerDigital::right);
    okapi::ControllerButton toggleFieldCentricControlButton(okapi::ControllerDigital::left);
    bool isFieldCentric = false;

    // Task to handle state machine during auton
    void chassisTask(void *ign)
    {
        ChassisState currentState = SKIP;
        MTTContainer currentContainer;

        printf("Starting chassisTask loop\n");
        while (true)
        {
            chassisMutex->take(10);
            currentState = state;
            currentContainer = mttContainer;
            chassisMutex->give();

            //printf("chassisTask: got necessary variables\n");

            switch (currentState)
            {
                case OFF:
                    strafeVector(0.0, 0.0, 0.0);
                    break;
                case SKIP:
                    break;
                case ROCK:
                    strafeVector(okapi::pi / 2, 0.0, 1.0);
                    delay(200);
                    strafeVector(-okapi::pi / 2, 0.0, 1.0);
                    delay(200);
                case MTT:
                    printf("chassisTask: in case MTT\n");
                    // Convert to nearest equivalent angle
                    currentContainer.theta = nearestEquivalentAngle(currentContainer.theta.getValue(), chassisController->getState().theta.getValue()) * 1_rad;

                    // Initialize PID controllers
                    auto distanceController = okapi::IterativeControllerFactory::posPID(0.35, 0.0015, 0.015);
                    auto thetaController = okapi::IterativeControllerFactory::posPID(3.0, 4.0, 0.07);
                    auto lineUpController = okapi::IterativeControllerFactory::posPID(0.1, 0.0, 0.0);
                    distanceController.setTarget(0);
                    thetaController.setTarget(currentContainer.theta.getValue());
                    double convertedTargetTheta = okapi::pi / 2 - currentContainer.theta.getValue();
                    lineUpController.setTarget(currentContainer.theta.getValue());

                    //printf("~~~~~~~~~~~~~~~MTT~~~~~~~~~~~~~~~\ntX: %3.3f\ttY: %3.3f\ttT: %3.3f\tmaxS: %3.3f\tmaxO: %3.3f\n",
                            //targetX.convert(okapi::inch), targetY.convert(okapi::inch), targetTheta.convert(okapi::degree), maxSpeed, maxOmega);

                    // Initialize SettledUtils depending on park
                    std::unique_ptr<okapi::SettledUtil> distanceSettled;
                    std::unique_ptr<okapi::SettledUtil> thetaSettled;
                    if (currentContainer.park)
                    {
                        distanceSettled = std::make_unique<okapi::SettledUtil>(std::make_unique<okapi::Timer>(), 1.0, 0.05, 200_ms);
                        thetaSettled = std::make_unique<okapi::SettledUtil>(std::make_unique<okapi::Timer>(), 0.05, 0.005, 200_ms);
                    }
                    else
                    {
                        distanceSettled = std::make_unique<okapi::SettledUtil>(std::make_unique<okapi::Timer>(), 3.0, 100, 0_ms);
                        thetaSettled = std::make_unique<okapi::SettledUtil>(std::make_unique<okapi::Timer>(), 0.2, 100, 0_ms);
                    }

                    //printf("Starting MTT loop\n");

                    int mttTimer = millis();
                    
                    while (true)
                    {
                        if (getState() != MTT)
                            break;
                        
                        // Calculate current states
                        okapi::OdomState currentState = chassisController->getState();
                        double xDiff = currentContainer.x.convert(okapi::inch) - currentState.x.convert(okapi::inch);
                        double yDiff = currentContainer.y.convert(okapi::inch) - currentState.y.convert(okapi::inch);

                        // Calculate errors
                        double distanceError = std::sqrt(std::pow(xDiff, 2) + std::pow(yDiff, 2));
                        double thetaError = currentContainer.theta.getValue() - currentState.theta.getValue();

                        // Check if settled
                        if (distanceSettled->isSettled(distanceError) && thetaSettled->isSettled(thetaError))
                        {
                            setState(OFF);
                            break;
                        }

                        // Calculate targets based on PID and heading
                        double targetHeading = std::atan2(yDiff, xDiff) + currentState.theta.getValue();
                        double targetSpeed = -distanceController.step(distanceError) * currentContainer.maxSpeed;
                        double targetOmega = thetaController.step(currentState.theta.getValue()) * currentContainer.maxOmega;

                        double headingDelta = 0;
                        double originalHeading = targetHeading;
                        if (currentContainer.lineUp && distanceError > 6)
                        {
                            targetHeading = nearestEquivalentAngle(targetHeading, convertedTargetTheta);
                            headingDelta = -lineUpController.step(targetHeading) * okapi::pi / 2;
                            targetHeading += headingDelta;
                            if (targetHeading - convertedTargetTheta > okapi::pi / 2)
                                targetHeading = convertedTargetTheta + okapi::pi / 2;
                            if (targetHeading - convertedTargetTheta < -okapi::pi / 2)
                                targetHeading = convertedTargetTheta - okapi::pi / 2;
                        }

                        // Power motors based on targets
                        if (targetSpeed < 0.7)
                            moveVector(targetHeading, targetOmega, targetSpeed);
                        else
                            strafeVector(targetHeading, targetOmega, targetSpeed);

                        // Logging
                        if (millis() - mttTimer > 100)
                        {
                            mttTimer = millis();
                            //printf("ASYNC xDiff: %3.3f\tyDiff: %3.3f\tdE: %3.3f\ttE: %3.3f\theading: %3.3f\tspeed: %3.3f\tomega: %3.3f\n", xDiff, yDiff, distanceError, thetaError, targetHeading * okapi::radianToDegree, targetSpeed, targetOmega);
                            printf("ASYNC xDiff: %3.3f\tyDiff: %3.3f\toriginalHeading: %3.3f\theadingDelta: %3.3f\theading: %3.3f\n", xDiff, yDiff, originalHeading * okapi::radianToDegree, headingDelta * okapi::radianToDegree, targetHeading * okapi::radianToDegree);
                        }

                        delay(10);
                    }
                    break;
            }
            delay(10);
        }
    }
    void init()
    {
        if (!initialized)
        {
            printf("Initializing chassisTask\n");
            chassisMutex = std::make_unique<Mutex>();
            delay(20);
            
            chassisTaskhandler = std::make_unique<Task>(chassisTask, nullptr, TASK_PRIORITY_DEFAULT);
            initialized = true;
            printf("Done initializing chassisTask\n");
        }
        else
        {
            chassisTaskhandler->resume();
        }
    }
    void stop()
    {
        if (chassisTaskhandler)
            chassisTaskhandler->suspend();
    }
    void setState(ChassisState newState)
    {
        chassisMutex->take(10);
        state = newState;
        chassisMutex->give();
    }
    ChassisState getState()
    {
        ChassisState currentState = OFF;
        chassisMutex->take(50);
        currentState = state;
        chassisMutex->give();
        return currentState;
    }
    int waitUntilSettled(int timeout)
    {
        int timeoutTimer = millis();
        while (true)
        {
            if (getState() != MTT)
                break;

            if (millis() - timeoutTimer > timeout)
            {
                setState(OFF);
                return -1;
            }
            delay(50);
        }
        return 0;
    }
    int waitUntilSettled()
    {
        return waitUntilSettled(60000);
    }
    int waitUntilStuck(int timeout)
    {
        okapi::SettledUtil distanceStuckUtil(std::make_unique<okapi::Timer>(), 2.0, 2.0, 400_ms);
        okapi::SettledUtil thetaStuckUtil(std::make_unique<okapi::Timer>(), 0.2, 0.2, 400_ms);
        int timeoutTimer = millis();

        while (true)
        {
            if (getState() != MTT)
                return 0;
            if (millis() - timeoutTimer > timeout)
            {
                setState(OFF);
                return -1;
            }
            okapi::OdomState currentV = getVelocity();
            double robotV = std::sqrt(std::pow(currentV.x.convert(okapi::inch), 2) + std::pow(currentV.y.convert(okapi::inch), 2));
            //printf("robotV: %3.3f\trobotOmega: %3.3f\n", robotV, currentV.theta.getValue());
            if (distanceStuckUtil.isSettled(robotV) && thetaStuckUtil.isSettled(currentV.theta.getValue()))
            {
                setState(OFF);
                return -2;
            }
        }
    }
    okapi::OdomState getVelocity()
    {
        okapi::OdomState prevState = chassisController->getState();
        delay(50);
        okapi::OdomState currentState = chassisController->getState();
        currentState.x = (currentState.x - prevState.x) / (50.0 / 1000.0);
        currentState.y = (currentState.y - prevState.y) / (50.0 / 1000.0);
        currentState.theta = (currentState.theta - prevState.theta) / (50.0 / 1000.0);
        //printf("getV: X: %3.3f\tY: %3.3f\tT: %3.3f\n", currentState.x.convert(okapi::inch),currentState.y.convert(okapi::inch), currentState.theta.getValue());
        return currentState;
    }
    void moveToTargetAsync(okapi::QLength targetX, okapi::QLength targetY, okapi::QAngle targetTheta, double maxSpeed, double maxOmega, bool park)
    {
        moveToTargetAsync(targetX, targetY, targetTheta, maxSpeed, maxOmega, park, false);
    }
    void moveToTargetAsync(okapi::QLength targetX, okapi::QLength targetY, okapi::QAngle targetTheta, double maxSpeed, double maxOmega, bool park, bool lineUp)
    {
        setState(OFF);
        delay(50);
        // Set variables in container and update chassis state
        chassisMutex->take(10);
        mttContainer.x = targetX;
        mttContainer.y = targetY;
        mttContainer.theta = targetTheta;
        mttContainer.maxSpeed = maxSpeed;
        mttContainer.maxOmega = maxOmega;
        mttContainer.park = park;
        mttContainer.lineUp = lineUp;
        state = MTT;
        chassisMutex->give();
    }
    void moveToTarget(okapi::QLength targetX, okapi::QLength targetY, okapi::QAngle targetTheta, double maxSpeed, double maxOmega, bool park)
    {
        moveToTargetAsync(targetX, targetY, targetTheta, maxSpeed, maxOmega, park);
        waitUntilSettled();
    }
    double nearestEquivalentAngle(double angle, double reference)
    {
        return std::round((reference - angle) / (2 * okapi::pi)) * 2 * okapi::pi + angle;
    }
    void setOdomState(okapi::QLength newX, okapi::QLength newY, okapi::QAngle newTheta)
    {
        okapi::OdomState newState;
        newState.x = newX;
        newState.y = newY;
        newState.theta = newTheta;
        chassisController->setState(newState);
    }
    okapi::OdomState getOdomState()
    {
        return chassisController->getState();
    }
    void resetOdom()
    {
        setOdomState(0_in, 0_in, 0_rad);
    }
    void strafeVector(double theta, double omega, double speed)
    {
        // formulas from https://www.desmos.com/calculator/qro9op4rmu
        double p1 = -std::cos(theta + okapi::pi/4);
        double p2 = std::sin(theta + okapi::pi/4);
        double s = std::max(std::abs(p1), std::abs(p2)) / speed;

        int topLeftVoltage = 12000 * ((p2 / s) * (1 - std::abs(omega)) + omega * speed);
        int topRightVoltage = 12000 * ((p1 / s) * (1 - std::abs(omega)) - omega * speed);
        int bottomRightVoltage = 12000 * ((p2 / s) * (1 - std::abs(omega)) - omega * speed);
        int bottomLeftVoltage = 12000 * ((p1 / s) * (1 - std::abs(omega)) + omega * speed);

        topLeft.moveVoltage(topLeftVoltage);
        topRight.moveVoltage(topRightVoltage);
        bottomRight.moveVoltage(bottomRightVoltage);
        bottomLeft.moveVoltage(bottomLeftVoltage);
    }
    void moveVector(double theta, double omega, double speed)
    {
        // formulas from https://theol0403.github.io/7842F-Programming-Journal/2019-11-20/odom-x-controller/
        double p1 = -std::cos(theta + okapi::pi/4);
        double p2 = std::sin(theta + okapi::pi/4);

        int topLeftVoltage = 12000 * (p2 * speed + omega);
        int topRightVoltage = 12000 * (p1 * speed - omega);
        int bottomRightVoltage = 12000 * (p2 * speed - omega);
        int bottomLeftVoltage = 12000 * (p1 * speed + omega);

        topLeft.moveVoltage(topLeftVoltage);
        topRight.moveVoltage(topRightVoltage);
        bottomRight.moveVoltage(bottomRightVoltage);
        bottomLeft.moveVoltage(bottomLeftVoltage);
    }

    // variables for debugging
    int logTimer = millis();
    bool isLogging = true;

    // variables for rocking
    int rockTimer = millis();
    bool rockForward = true;

    ADIEncoder leftEncoder{'A', 'B'};
    ADIEncoder rightEncoder{'C', 'D'};
    ADIEncoder backEncoder{'E', 'F', true};

    void opcontrol()
    {
        // Get values from controller
        int x = master.get_analog(ANALOG_LEFT_X);
        int y = master.get_analog(ANALOG_LEFT_Y);
        int a = master.get_analog(ANALOG_RIGHT_X);

        // Handle overrides
        if (strafeHorizontalButton.isPressed())
            y = 0;
        else if (strafeVerticalButton.isPressed())
            x = 0;
        if (toggleFieldCentricControlButton.changedToPressed())
            isFieldCentric = !isFieldCentric;
        
        // Calculate if driving is field centric or not
        double offset = (isFieldCentric) ? chassisController->getState().theta.getValue() : 0;

        double theta = std::atan2(y, x) + offset;
        double omega = a / (double) 127;
        double speed = std::sqrt(std::pow(x / (double) 127, 2) + std::pow(y / (double) 127, 2));

        //printf("theta: %1.3f\tomega: %1.3f\tspeed: %1.3f\n", theta, omega, speed);
        //speed = (speed > 1.0) ? 1.0 : speed;

        //moveVector(theta, omega, speed);

        if (intakePositiveButton.isPressed() && intakeNegativeButton.isPressed() && millis())
        {
            if (rockForward)
            {
                theta = okapi::pi / 2;
                omega = 0;
                speed = 1.0;
            }
            else
            {
                theta = -okapi::pi / 2;
                omega = 0;
                speed = 1.0;
            }
            if (millis() - rockTimer > 200)
            {
                rockForward = !rockForward;
                rockTimer = millis();
            }
        }

        drive->xArcade(speed * std::cos(theta), speed * std::sin(theta), omega);
        
        if (resetOdomButton.changedToPressed())
            resetOdom();

        // Logging
        if (millis() - logTimer > 150 && isLogging)
        {
            okapi::OdomState state = chassisController->getState();
            printf("X: %3.3f\tY: %3.3f\tT: %3.3f\t", state.x.convert(okapi::inch), state.y.convert(okapi::inch), state.theta.convert(okapi::degree));
            //odom::pose robotPose = odom::getPose();
			//printf("X*: %3.3f\tY*: %3.3f\tT*: %3.3f\t", robotPose.x, robotPose.y, robotPose.theta * okapi::radianToDegree);
			printf("L: %d\tR: %d\tB: %d\n", leftEncoder.get_value(), rightEncoder.get_value(), backEncoder.get_value());
			logTimer = millis();
        }
    }
}