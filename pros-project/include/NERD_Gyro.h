#ifndef NERD_GYRO_H
#define NERD_GYRO_H

#include "main.h"

struct gyro_config
{
	float std_deviation;
	float avg;
	float volts_per_degree_per_second;
	char gyro_flipped;
};

typedef struct
{
	struct gyro_config config;
	int port_number;
  pros::ADIAnalogIn analog_in;
} Gyro;

void gyro_calibrate (Gyro gyro);
void gyro_init (Gyro gyro, int port_number, char gyro_flipped);
float gyro_get_rate (Gyro gyro);

#endif
