#ifndef SONAR_H
#define SONAR_H

#include "main.h"

#define FIELD_WIDTH 140.5
#define LEFT_SONAR_TO_CENTER 8.75

double sonarReadFiltered(Ultrasonic sonar, double minValue, double maxValue, unsigned long minTime, unsigned long maxTime);

#endif
