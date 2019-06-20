#include "main.h"

double maxV = 0.0;
double maxA = 0.0;
double maxJ = 0.0;

void testMaxVAJ(void *ignore)
{
	double prevLeftV = leftWheelVelocity;
	double prevRightV = rightWheelVelocity;

	double prevLeftA = 0.0;
	double prevRightA = 0.0;

	double prevLeftJ = 0.0;
	double prevRightJ = 0.0;

	unsigned long timer = millis();

	while (true)
	{
		powerMotors(127, 127);

		unsigned long deltaTimeMS = millis() - timer;
		timer = millis();

		double currentLeftV = leftWheelVelocity;
		double currentRightV = rightWheelVelocity;

		double currentLeftA = 0.0;
		double currentRightA = 0.0;

		double currentLeftJ = 0.0;
		double currentRightJ = 0.0;

		if (deltaTimeMS != 0)
		{
			double deltaTime = ((double) deltaTimeMS) * 0.001;

			currentLeftA = (currentLeftV - prevLeftV) / deltaTime;
			currentRightA = (currentRightV - prevRightV) / deltaTime;

			currentLeftJ = (currentLeftA - prevLeftA) / deltaTime;
			currentRightJ = (currentRightA - prevRightA) / deltaTime;
		}

		prevLeftV = currentLeftV;
		prevRightV = currentRightV;

		prevLeftA = currentLeftA;
		prevRightA = currentRightA;

		prevLeftJ = currentLeftJ;
		prevRightJ = currentRightJ;


		mutexTake(mutexes[MUTEX_POSE], -1);
		maxV = fmax(maxV, fmin(currentLeftV, currentRightV));
		maxA = fmax(maxA, fmin(currentLeftA, currentRightA));
		maxJ = fmax(maxJ, fmin(currentLeftJ, currentRightJ));
		mutexGive(mutexes[MUTEX_POSE]);

		printf("V: %f\tA: %f\tJ: %f\n", maxV, maxA, maxJ);

		delay(20);
	}
}
