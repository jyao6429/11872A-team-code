#include "main.h"
#include <algorithm>
#include <cmath>

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

    // override buttons
    okapi::ControllerButton strafeHorizontalButton(okapi::ControllerDigital::A);
	okapi::ControllerButton strafeVerticalButton(okapi::ControllerDigital::B);
	okapi::ControllerButton resetOdomButton(okapi::ControllerDigital::right);

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
        {
            okapi::OdomState zeroState;
            chassisController->setState(zeroState);
        }

        okapi::OdomState state = chassisController->getState();
        printf("X: %3.3f\tY: %3.3f\tT: %3.3f\n", state.x.convert(okapi::inch), state.y.convert(okapi::inch), state.theta.convert(okapi::degree));
    }
}