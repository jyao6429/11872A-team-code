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
void resetAgainstWall()
{

}
