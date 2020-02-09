#include "main.h"

// Drive motor ports
const int PORT_leftMotor0 = 2;
const int PORT_leftMotor1 = 3;
const int PORT_rightMotor0 = 9;
const int PORT_rightMotor1 = 8;

std::shared_ptr<ChassisController> chassis;

void initDrive()
{
  chassis = ChassisControllerBuilder()
						.withMotors({PORT_leftMotor0, PORT_leftMotor1}, {-PORT_rightMotor0, -PORT_rightMotor1})
						.withDimensions(AbstractMotor::gearset::green, {{4_in, 15.5_in}, imev5GreenTPR})
						.build();
}
void setDrive(double leftPower, double rightPower)
{
  chassis->getModel()->tank(leftPower, rightPower);
}
void setDrive(int leftPower, int rightPower)
{
  chassis->getModel()->tank((double) leftPower / 127.0, (double) rightPower / 127.0);
}
void setDriveVel(double leftSpeed, double rightSpeed)
{
  chassis->getModel()->left(leftSpeed);
  chassis->getModel()->right(rightSpeed);
}
void stopDrive()
{
  chassis->getModel()->tank(0, 0);
}
