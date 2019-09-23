#ifndef GLOBAL_H
#define GLOBAL_H

#include "main.h"

// Left-back motor port
extern const int PORT_leftMotor0;
// Left-front motor port
extern const int PORT_leftMotor1;
// Right-back motor port
extern const int PORT_rightMotor0;
// Right-front motor port
extern const int PORT_rightMotor1;

// Other motor ports
extern const int PORT_tilter;
extern const int PORT_leftArm;
extern const int PORT_rightArm;
extern const int PORT_leftRollers;
extern const int PORT_rightRollers;

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

// Potentiometer port
extern const int PORT_liftPot;

// Ultrasonic ECHO port
extern const int PORT_leftSonarOrange;
// Ultrasonic PING port
extern const int PORT_leftSonarYellow;

// Left-side ultrasonic sensor
extern Ultrasonic leftSonar;

// startTesting button
extern const int PORT_startTestButton;
// stopTesting limit switch
extern const int PORT_stopTestButton;

// Limit switch when tray is vertical
extern const int PORT_verticalTilterLimit;
// Limit switch when tray is at maximum angle
extern const int PORT_angledTilterLimit;

// Enum for different mutexes
enum MutexTypes
{
  MUTEX_POSE,
  MUTEX_ASYNC
};
// Mutexes
Mutex mutexes[2];

#endif
