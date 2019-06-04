// Rewritten for PROS CORTEX by jyao6429, original license is below
/* -----------------------------------------------------------------------------
                                  MIT License

                        Copyright (c) 2018 Jason McKinney

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.

--------------------------------------------------------------------------------
	NERD_Gyro.c

	Created:  2016-06-30

	Minor Revisions:
	-	v1.0.0  Initial Release

--------------------------------------------------------------------------------
	The author asks that proper attribution be given for this software should the
	source be unavailable (for example, if compiled into a binary/used on a robot).

	The author can be contacted via email at jason_at_jmmckinney_dot_net
	or on the VEX Forum as jmmckinney.
-------------------------------------------------------------------------------- */

#include "main.h"

// ignore data within n standard deviations of no motion average
#define GYRO_STD_DEVS 5

#define GYRO_OVERSAMPLE 2

// points or time in mSec that the gyro calibrates for
#define GYRO_CALIBRATION_POINTS 1500

double calibration_buffer[GYRO_CALIBRATION_POINTS];

int getGyroAnalog(struct NERD_Gyro *gyro)
{
	return analogRead(gyro->port_number);
}
void gyro_calibrate (struct NERD_Gyro *gyro)
{
	double raw_average = 0.0;
	double std_deviation = 0.0;

	print("Starting Calibration\n");
	// calculate average gyro reading with no motion
	for (int i = 0; i < GYRO_CALIBRATION_POINTS; ++i)
  {
		double raw = (double) getGyroAnalog(gyro);
		//printf("%f\n", raw);
		raw_average += raw;
		calibration_buffer[i] = raw;
		delay(1);
	}
	raw_average /= GYRO_CALIBRATION_POINTS;
	printf("RAW_AVG: %f\n", raw_average);
	gyro->config.avg = raw_average;

	//calcuate the standard devation, or the average distance
	//from the average on the data read
	for (int i = 0; i < GYRO_CALIBRATION_POINTS; ++i)
  {
		std_deviation += fabs (raw_average - calibration_buffer[i]);
  }
	std_deviation /= (double) GYRO_CALIBRATION_POINTS;

printf("STD_DEV: %f\n", std_deviation);
	gyro->config.std_deviation = std_deviation;

	/*
	 * Datasheet from VEX indicates that the sensitivity of the gyro is 1.1mV/dps
	 * and the cortex ADC for raw analog reads ranges from 0-4095 for 0v-5v
	 * readings. The gyro is scaled from the nominal 2.7v-3.6v operating range
	 * that the actual chip has to work on the cortex's 5v logic voltage. The scale multiplier
	 * value is in the ballpark of 1.515 (tuned to 1.493 by jyao6429).
	 */
	gyro->config.volts_per_degree_per_second = 0.0011 * 1.515;
}

void gyro_init (struct NERD_Gyro *gyro, int port_number, bool gyro_flipped)
{
	gyro->port_number = port_number;
	gyro->config.gyro_flipped = gyro_flipped;
	gyro_calibrate (gyro);
}

double gyro_get_rate (struct NERD_Gyro *gyro)
{
	double gyro_read = 0.0;

	#ifdef GYRO_OVERSAMPLE
		if (GYRO_OVERSAMPLE > 0)
    {
			int sample_sum = 0;
			int n_samples = pow(4, GYRO_OVERSAMPLE);

			for (int i = 0; i < n_samples; ++i)
      {
				sample_sum += getGyroAnalog(gyro);
      }
			gyro_read = (double) sample_sum / (double) n_samples;
		}
		else
    {
			gyro_read = (double) getGyroAnalog(gyro);
    }
	#else
		gyro_read = (double) getGyroAnalog(gyro);
	#endif

	//Difference from zero-rate value or the average calibration read
	double difference = gyro_read - gyro->config.avg;

	//Difference from zero-rate value, in volts
	double gyro_voltage = difference * 5.0 / 4095.0;

	if (fabs (difference) > GYRO_STD_DEVS * gyro->config.std_deviation)
  {
		if (gyro->config.gyro_flipped)
    {
			return -1 * gyro_voltage / gyro->config.volts_per_degree_per_second;
    }
		else
    {
			return gyro_voltage / gyro->config.volts_per_degree_per_second;
    }
  }
	return 0;
}
