#ifndef SONAR_H
#define SONAR_H

#include "main.h"

// Total width of the field in inches
#define FIELD_WIDTH 140.5
// Distance from the left ultrasonic sensor to the center of the robot
#define LEFT_SONAR_TO_CENTER 8.75
// Distance from the back of the robot to the center of the robot
#define BACK_TO_CENTER 8.75

// Enum for resetting on different walls
typedef enum ResetType
{
  RESET_X_NEAR,
  RESET_Y_NEAR,
  RESET_X_FAR,
  RESET_Y_FAR
} ResetType;

/**
 * Reads a filtered ultrasonic value in inches
 *
 * @param sonar - the ultrasonic sensor to read
 * @param minValue - the minimum value the sensor should read in inches
 * @param maxValue - the maximum value the sensor should read in inches
 * @param minTime - the minimum amount of time to collect readings in milliseconds
 * @param maxTime - the maximum amount of time to collect readings in milliseconds
 *
 * @returns an averaged reading between the min and max values, or ULTRA_BAD_RESPONSE if not enough good readings
 */
double sonarReadFiltered(Ultrasonic sonar, double minValue, double maxValue, unsigned long minTime, unsigned long maxTime);
/**
 * Resets the robot's pose by driving back into a way and using the left ultrasonic sensor
 *
 * @param resetType - signifies which wall the robot is backing up into
 * @param minDisFromWall - the minimum distance the robot should be from the wall in inches
 * @param maxDisFromWall - the maximum distance the robot should be from the wall in inches
 * @param minTime - the minimum amount of time to collect readings in milliseconds
 * @param maxTime - the maximum amount of time to collect readings in milliseconds
 */
void resetLeftAgainstWall(ResetType resetType, double minDisFromWall, double maxDisFromWall, unsigned long minTime, unsigned long maxTime);

#endif
