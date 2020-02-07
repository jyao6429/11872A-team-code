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
extern const int PORT_arm;
extern const int PORT_leftRoller;
extern const int PORT_rightRoller;

// Motors themselves
Motor trayMotor;
Motor armMotor;
Motor leftIntakeMotor;
Motor rightIntakeMotor;

// Controller
Controller controller;

// Chassis controller
auto chassis;

// Left encoder port
extern const char PORT_leftEncoder;
// Right encoder port
extern const char PORT_rightEncoder;
// Back encoder port
extern const char PORT_backEncoder;

// Left encoder
extern ADIEncoder leftEncoder;
// Right encoder
extern ADIEncoder rightEncoder;
// Back encoder
extern ADIEncoder backEncoder;

// Potentiometer port for the tray
extern const char PORT_trayPot;

Potentiometer trayPot;

// Boolean to store if the main controller is connected
extern bool isMainConnected;
// Boolean to store if the partner controller is connected
extern bool isPartnerConnected;

enum MutexTypes
{
  MUTEX_POSE,
  MUTEX_ASYNC_CHASSIS,
  MUTEX_ASYNC_TRAY,
  MUTEX_ASYNC_ARM
};
// Mutexes
pros::Mutex mutexes[4];

#endif
