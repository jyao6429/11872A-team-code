#include "main.h"

// Drive motor ports
const int PORT_leftBackMotor = 1;
const int PORT_leftFrontMotor = 2;
const int PORT_rightBackMotor = 3;
const int PORT_rightFrontMotor = 4;

// Other motor ports

// Encoder ports
const int PORT_leftEncoder = 1;
const int PORT_rightEncoder = 3;
const int PORT_backEncoder = 1;

// Encoders themselves
Encoder leftEncoder;
Encoder rightEncoder;
Encoder backEncoder;

// Sensor ports
const int PORT_gyro = 1;
const int PORT_liftPot = 2;

// Gyro itself
NERD_Gyro gyro;
