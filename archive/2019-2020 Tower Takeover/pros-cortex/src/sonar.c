#include "main.h"

double sonarReadFiltered(Ultrasonic sonar, double minValue, double maxValue, unsigned long minTime, unsigned long maxTime)
{
  // Convert min and max values to centimeters
  minValue = inToCM(minValue);
  maxValue = inToCM(maxValue);

  // Variables to store gathered data
  int totalGoodSum = 0;
  int totalGoodCount = 0;

  // Variables to store cutoff times
  unsigned long minCutoffTime = millis() + minTime;
  unsigned long maxCutoffTime = millis() + maxTime;

  while (millis() < minCutoffTime || totalGoodCount < 10)
  {
    // Return if failed to get good readings within the time interval
    if (millis() > maxCutoffTime)
      return ULTRA_BAD_RESPONSE;

    // Get the current readout and check if it is a valid response
    int currentRead = ultrasonicGet(sonar);

    // Add to good readings if within range
    if (currentRead < maxValue && currentRead > minValue && currentRead != ULTRA_BAD_RESPONSE)
    {
      totalGoodSum += currentRead;
      totalGoodCount++;
    }
  }
  // Return the averaged value
  return cmToIN(totalGoodSum) / (double) totalGoodCount;
}
void resetLeftAgainstWall(ResetType resetType, double minDisFromWall, double maxDisFromWall, unsigned long minTime, unsigned long maxTime)
{
  // Variables to make sure robot is fully against wall
  unsigned long resetTimer = millis();
  unsigned long stopTimer = millis();
  bool isStopped = false;

  // Keep driving backwards until robot is still for 350 milliseconds
  while (!isStopped)
  {
    setDriveLinear(-30, -30);

    if (!isRobotStopped())
      stopTimer = millis();

    if (millis() - stopTimer > 350)
    {
      isStopped = true;
      setDrive(-5, -5);
    }
  }

  // Get a filtered value
  double filteredRead = sonarReadFiltered(leftSonar, minDisFromWall, maxDisFromWall, minTime, maxTime);

  // Add on distance to center of the robot
  if (filteredRead > 0)
    filteredRead += LEFT_SONAR_TO_CENTER;

  printf("filteredRead: %3.3f\n", filteredRead);

  // Switch between the different walls
  switch (resetType)
  {
    case RESET_X_NEAR:
      if (filteredRead > 0)
        resetPositionFull(&globalPose, filteredRead, BACK_TO_CENTER, 0.0, false);
      else
        resetPositionFull(&globalPose, globalPose.x, BACK_TO_CENTER, 0.0, false);
      break;
    case RESET_Y_NEAR:
      if (filteredRead > 0)
        resetPositionFull(&globalPose, BACK_TO_CENTER, FIELD_WIDTH - filteredRead, M_PI / 2, false);
      else
        resetPositionFull(&globalPose, BACK_TO_CENTER, globalPose.y, M_PI / 2, false);
      break;
    case RESET_X_FAR:
      if (filteredRead > 0)
        resetPositionFull(&globalPose, FIELD_WIDTH - filteredRead, FIELD_WIDTH - BACK_TO_CENTER, M_PI, false);
      else
        resetPositionFull(&globalPose, globalPose.x, FIELD_WIDTH - BACK_TO_CENTER, M_PI, false);
      break;
    case RESET_Y_FAR:
      if (filteredRead > 0)
        resetPositionFull(&globalPose, FIELD_WIDTH - BACK_TO_CENTER, filteredRead, -M_PI / 2, false);
      else
        resetPositionFull(&globalPose, FIELD_WIDTH - BACK_TO_CENTER, globalPose.y, -M_PI / 2, false);
      break;
  }
  // Reset the velocity as well
  resetVelocity(&globalVel, globalPose);
  stopDrive();

  printf("Reset position in %d ms:   X: %3.3f   Y: %3.3f   A: %3.3f\n", (int) (millis() - resetTimer), globalPose.x, globalPose.y, radToDeg(globalPose.angle));
}
