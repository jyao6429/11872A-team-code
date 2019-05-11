#ifndef NERD_GYRO_H
#define NERD_GYRO_H

struct gyro_config{
	float std_deviation;
	float avg;
	float volts_per_degree_per_second;
	char gyro_flipped;
};

typedef struct {
	struct gyro_config config;
	int port_number;
} Gyro;

#endif
