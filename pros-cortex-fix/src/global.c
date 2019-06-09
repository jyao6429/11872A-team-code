#include "main.h"

// Drive motor ports
const int PORT_leftBackMotor = 9;
const int PORT_leftFrontMotor = 8;
const int PORT_rightBackMotor = 2;
const int PORT_rightFrontMotor = 3;

// Other motor ports

// Encoder ports
const int PORT_leftEncoder = 5;
const int PORT_rightEncoder = 1;
const int PORT_backEncoder = 3;

// Encoders themselves
Encoder leftEncoder;
Encoder rightEncoder;
Encoder backEncoder;

// Sensor ports
const int PORT_liftPot = 2;

// Mutexes
Mutex mutexes[1];
