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
        .build();

    void moveVector(double theta, double omega, double speed)
    {
        // formulas from https://www.desmos.com/calculator/qro9op4rmu
        double p1 = -std::cos(theta + pi/4);
        double p2 = std::sin(theta + pi/4);
        double s = std::max(std::abs(p1), std::abs(p2)) / speed;

        int topLeftVoltage = 12000 * ((p2 / s) * (1 - std::abs(omega)) + omega * speed));
        int topRightVoltage = 12000 * ((p1 / s) * (1 - std::abs(omega)) - omega * speed));
        int bottomRightVoltage = 12000 * ((p1 / s) * (1 - std::abs(omega)) + omega * speed));
        int bottomLeftVoltage = 12000 * ((p2 / s) * (1 - std::abs(omega)) - omega * speed));

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

        double theta = std::atan2(y, x);
        double omega = a / (double) 127;
        double speed = std::sqrt(std::pow(x / (double) 127, 2) + std::pow(y / (double) 127, 2));
        speed = (speed > 1.0) ? 1.0 : speed;

        moveVector(theta, omega, speed);
    }
}