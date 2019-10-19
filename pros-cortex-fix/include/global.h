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
extern const int PORT_tray;
extern const int PORT_leftArm;
extern const int PORT_rightArm;
extern const int PORT_leftRoller;
extern const int PORT_rightRoller;

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
extern const int PORT_armPot;

// Ultrasonic ECHO port
extern const int PORT_leftSonarOrange;
// Ultrasonic PING port
extern const int PORT_leftSonarYellow;

// Left-side ultrasonic sensor
extern Ultrasonic leftSonar;

// startTesting button
extern const int PORT_interactButton;

// Limit switch when tray is vertical
//extern const int PORT_verticalTrayLimit;
// Limit switch when tray is at maximum angle
//extern const int PORT_angledTrayLimit;

// Enum for different mutexes
enum MutexTypes
{
  MUTEX_POSE,
  MUTEX_ASYNC_CHASSIS,
  MUTEX_ASYNC_TRAY,
  MUTEX_ASYNC_ARM
};
// Mutexes
Mutex mutexes[4];

#endif
