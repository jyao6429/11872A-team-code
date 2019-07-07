#ifndef GLOBAL_H
#define GLOBAL_H

#include "main.h"

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

// Enum for different mutexes
enum MutexTypes
{
  MUTEX_POSE,
  MUTEX_ASYNC
};
// Mutexes
Mutex mutexes[2];

#endif
