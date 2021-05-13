#include "main.h"
#include "okapi/api/odometry/odomState.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "pros/rtos.h"
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
        .withOdometry({{2.75_in, 15.162_in, 15.162_in / 2, 2.75_in}, okapi::quadEncoderTPR}, okapi::StateMode::CARTESIAN)
        .buildOdometry();
    
    std::shared_ptr<okapi::XDriveModel> drive = std::dynamic_pointer_cast<okapi::XDriveModel>(chassisController->getModel());

    // MTT variables
    bool isSettled = true;


    // override buttons
    okapi::ControllerButton strafeHorizontalButton(okapi::ControllerDigital::A);
	okapi::ControllerButton strafeVerticalButton(okapi::ControllerDigital::B);
	okapi::ControllerButton resetOdomButton(okapi::ControllerDigital::right);

    void moveToTarget(okapi::QLength targetX, okapi::QLength targetY, okapi::QAngle targetTheta, double maxSpeed, double maxOmega, bool park)
    {
        isSettled = false;

        targetTheta = nearestEquivalentAngle(targetTheta.getValue(), chassisController->getState().theta.getValue()) * 1_rad;

        auto distanceController = okapi::IterativeControllerFactory::posPID(0.5, 0.0, 0.025);
        auto thetaController = okapi::IterativeControllerFactory::posPID(4.0, 0.0, 0.1);

        distanceController.setTarget(0);
        thetaController.setTarget(targetTheta.getValue());

        std::unique_ptr<okapi::SettledUtil> distanceSettled;
        std::unique_ptr<okapi::SettledUtil> thetaSettled;

        printf("~~~~~~~~~~~~~~~MTT~~~~~~~~~~~~~~~\n
                tX: %3.3f\ttY: %3.3f\ttT: %3.3f\tmaxS: %3.3f\tmaxO: %3.3f\n",
                targetX.convert(okapi::inch), targetY.convert(okapi::inch), targetTheta.convert(okapi::degree), maxSpeed, maxOmega);

        if (park)
        {
            distanceSettled = std::make_unique<okapi::SettledUtil>(std::make_unique<okapi::Timer>(), 1.0, 0.1, 250_ms);
            thetaSettled = std::make_unique<okapi::SettledUtil>(std::make_unique<okapi::Timer>(), 0.1, 0.005, 250_ms);
        }
        else
        {
            distanceSettled = std::make_unique<okapi::SettledUtil>(std::make_unique<okapi::Timer>(), 2.0, 100, 0_ms);
            thetaSettled = std::make_unique<okapi::SettledUtil>(std::make_unique<okapi::Timer>(), 0.2, 100, 0_ms);
        }

        printf("Starting MTT loop\n");

        int timer = millis();
        
        while (true)
        {
            okapi::OdomState currentState = chassisController->getState();
            double xDiff = targetX.convert(okapi::inch) - currentState.x.convert(okapi::inch);
            double yDiff = targetY.convert(okapi::inch) - currentState.y.convert(okapi::inch);

            double distanceError = std::sqrt(std::pow(xDiff, 2) + std::pow(yDiff, 2));
            double thetaError = targetTheta.getValue() - currentState.theta.getValue();

            if (distanceSettled->isSettled(distanceError) && thetaSettled->isSettled(thetaError))
            {
                isSettled = true;
                strafeVector(0, 0, 0);
                break;
            }

            double targetHeading = std::atan2(yDiff, xDiff) + currentState.theta.getValue();
            double targetSpeed = -distanceController.step(distanceError) * maxSpeed;
            double targetOmega = thetaController.step(currentState.theta.getValue()) * maxOmega;


            if (targetSpeed < 0.7)
            {
                //double targetX = targetSpeed * std::cos(targetHeading);
                //double targetY = targetSpeed * std::sin(targetHeading);
                //drive->xArcade(targetX, targetY, targetOmega);
                moveVector(targetHeading, targetOmega, targetSpeed);
            }
            else
            {
                strafeVector(targetHeading, targetOmega, targetSpeed);
            }

            if (millis() - timer > 100)
            {
                timer = millis();
                printf("xDiff: %3.3f\tyDiff: %3.3f\tdE: %3.3f\ttE: %3.3f\theading: %3.3f\tspeed: %3.3f\tomega: %3.3f\n", xDiff, yDiff, distanceError, thetaError, targetHeading * okapi::radianToDegree, targetSpeed, targetOmega);
                //printf("distanceDerivative: %3.3f\tthetaDerivative: %3.3f\n");
            }

            delay(10);
        }
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

    int timer = millis();
    bool isLogging = true;

    void opcontrol()
    {
        int x = master.get_analog(ANALOG_LEFT_X);
        int y = master.get_analog(ANALOG_LEFT_Y);
        int a = master.get_analog(ANALOG_RIGHT_X);

        if (strafeHorizontalButton.isPressed())
            y = 0;
        else if (strafeVerticalButton.isPressed())
            x = 0;

        //double theta = std::atan2(y, x);
        //double omega = a / (double) 127;
        //double speed = std::sqrt(std::pow(x / (double) 127, 2) + std::pow(y / (double) 127, 2));

        //printf("theta: %1.3f\tomega: %1.3f\tspeed: %1.3f\n", theta, omega, speed);
        //speed = (speed > 1.0) ? 1.0 : speed;

        //moveVector(theta, omega, speed);

        drive->xArcade(x / (double) 127, y / (double) 127, a / (double) 127);
        
        if (resetOdomButton.changedToPressed())
            resetOdom();

        if (millis() - timer > 100 && isLogging)
        {
            okapi::OdomState state = chassisController->getState();
            printf("X: %3.3f\tY: %3.3f\tT: %3.3f\n", state.x.convert(okapi::inch), state.y.convert(okapi::inch), state.theta.convert(okapi::degree));
            timer = millis();
        }
    }
}