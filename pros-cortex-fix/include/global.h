#ifndef GLOBAL_H
#define GLOBAL_H

#include "main.h"
#include "NERD_Gyro.h"

// Left-back motor port
extern const int PORT_leftBackMotor;
// Left-front motor port
extern const int PORT_leftFrontMotor;
// Right-back motor port
extern const int PORT_rightBackMotor;
// Right-front motor port
extern const int PORT_rightFrontMotor;

// Other motor ports

// Left encoder port
extern const int PORT_leftEncoder;
// Right encoder port
extern const int PORT_rightEncoder;
// Back encoder port
extern const int PORT_backEncoder;

// Left encoder
extern Encoder leftEncoder;
// Right encoder
extern Encoder rightEncoder;
// Back encoder
extern Encoder backEncoder;

// Gyro port
extern const int PORT_gyro;
// Potentiometer port
extern const int PORT_liftPot;

// Gyro itself
struct NERD_Gyro mainGyro;

#endif
