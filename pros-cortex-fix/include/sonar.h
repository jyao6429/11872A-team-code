#ifndef SONAR_H
#define SONAR_H

#include "main.h"

#define FIELD_WIDTH 140.5
#define LEFT_SONAR_TO_CENTER 8.75
#define BACK_TO_CENTER 8.75

typedef enum ResetType
{
  RESET_X_NEAR,
  RESET_Y_NEAR,
  RESET_X_FAR,
  RESET_Y_FAR
} ResetType;

double sonarReadFiltered(Ultrasonic sonar, double minValue, double maxValue, unsigned long minTime, unsigned long maxTime);
void resetAgainstWall(ResetType resetType, double minDisFromWall, double maxDisFromWall, unsigned long minTime, unsigned long maxTime);

#endif
