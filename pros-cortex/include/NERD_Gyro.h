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

void gyro_calibrate (NERD_Gyro gyro);
void gyro_init (NERD_Gyro gyro, int port_number, bool gyro_flipped);
double gyro_get_rate (NERD_Gyro gyro);

#endif
