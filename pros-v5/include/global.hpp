#ifndef GLOBAL_H
#define GLOBAL_H

#include "main.h"

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
extern pros::Mutex mutexes[4];

#endif
