#ifndef NERD_GYRO_H
#define NERD_GYRO_H

#include "main.h"

struct gyro_config
{
	double std_deviation;
	double avg;
	double volts_per_degree_per_second;
	bool gyro_flipped;
};

typedef struct
{
	struct gyro_config config;
	int port_number;
} NERD_Gyro;
/**
 * generate calibration data for the gyro by collecting
 * zero movement data for reference when reading data later
 *
 * @param gyro instance of gyro structure
 */
void gyro_calibrate (NERD_Gyro gyro);
/**
 * initialize gyro and run the calibration subroutine
 *
 * @param gyro instance of gyro structure
 * @param port_number the port number of the gyro
 */
void gyro_init (NERD_Gyro gyro, int port_number, bool gyro_flipped);
/**
 * calculate filtered gyro rate data, ignoring anything within
 * GYRO_STD_DEVS standard deviations of the average gyro
 * rate value at zero motion
 *
 * @param gyro instance of gyro structure
 *
 * @return gyro rate, in degrees per second
 */
double gyro_get_rate (NERD_Gyro gyro);

#endif
