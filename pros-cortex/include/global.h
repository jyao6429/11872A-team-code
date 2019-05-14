#ifndef GLOBAL_H
#define GLOBAL_H

#include "main.h"

// Drive motor ports
extern const int PORT_leftBackMotor;
extern const int PORT_leftFrontMotor;
extern const int PORT_rightBackMotor;
extern const int PORT_rightFrontMotor;

// Other motor ports

// Encoder ports
extern const int PORT_leftEncoder;
extern const int PORT_rightEncoder;
extern const int PORT_backEncoder;

// Encoders themselves
extern Encoder leftEncoder;
extern Encoder rightEncoder;
extern Encoder backEncoder;

// Sensor ports
extern const int PORT_gyro;
extern const int PORT_liftPot;

#endif
