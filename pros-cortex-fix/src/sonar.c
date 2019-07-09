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

    if (currentRead < maxValue && currentRead > minValue && currentRead != ULTRA_BAD_RESPONSE)
    {
      totalGoodSum += currentRead;
      totalGoodCount++;
    }
  }
  // Return the averaged value
  return cmToIN(totalGoodSum) / (double) totalGoodCount;
}
void resetAgainstWall(ResetType resetType, double minDisFromWall, double maxDisFromWall, unsigned long minTime, unsigned long maxTime)
{
  unsigned long resetTimer = millis();
  unsigned long stopTimer = millis();
  bool isStopped = false;

  while (!isStopped)
  {
    powerMotorsLinear(-30, -30);

    if (!isRobotStopped())
      stopTimer = millis();

    if (millis() - stopTimer > 350)
    {
      isStopped = true;
      powerMotors(-5, -5);
    }
  }

  double filteredRead = sonarReadFiltered(leftSonar, minDisFromWall, maxDisFromWall, minTime, maxTime);

  if ((int) filteredRead != ULTRA_BAD_RESPONSE)
    filteredRead += LEFT_SONAR_TO_CENTER;

  printf("filteredRead: %3.3f\n", filteredRead);

  switch (resetType)
  {
    case RESET_X_NEAR:
      if ((int) filteredRead != ULTRA_BAD_RESPONSE)
        resetPositionFull(&globalPose, filteredRead, BACK_TO_CENTER, 0.0, false);
      else
        resetPositionFull(&globalPose, globalPose.x, BACK_TO_CENTER, 0.0, false);
      break;
    case RESET_Y_NEAR:
      if ((int) filteredRead != ULTRA_BAD_RESPONSE)
        resetPositionFull(&globalPose, BACK_TO_CENTER, FIELD_WIDTH - filteredRead, M_PI / 2, false);
      else
        resetPositionFull(&globalPose, BACK_TO_CENTER, globalPose.y, M_PI / 2, false);
      break;
    case RESET_X_FAR:
      if ((int) filteredRead != ULTRA_BAD_RESPONSE)
        resetPositionFull(&globalPose, FIELD_WIDTH - filteredRead, FIELD_WIDTH - BACK_TO_CENTER, M_PI, false);
      else
        resetPositionFull(&globalPose, globalPose.x, FIELD_WIDTH - BACK_TO_CENTER, M_PI, false);
      break;
    case RESET_Y_FAR:
      if ((int) filteredRead != ULTRA_BAD_RESPONSE)
        resetPositionFull(&globalPose, FIELD_WIDTH - BACK_TO_CENTER, filteredRead, -M_PI / 2, false);
      else
        resetPositionFull(&globalPose, FIELD_WIDTH - BACK_TO_CENTER, globalPose.y, -M_PI / 2, false);
      break;
  }
  resetVelocity(&globalVel, globalPose);
  stopMotors();

  printf("Reset position in %d ms:   X: %3.3f   Y: %3.3f   A: %3.3f\n", (int) (millis() - resetTimer), globalPose.x, globalPose.y, radToDeg(globalPose.angle));
}
