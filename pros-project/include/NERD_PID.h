#ifndef NERD_PID_H
#define NERD_PID_H

/**
 * PID controller data structure
 */
typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float Kf;
	float sigma;
	float lastValue;
	unsigned long lastTime;
	float lastSetPoint;
} PID;

void pidInit (PID pid, float Kp, float Ki, float Kd);
void pidInit (PID pid, float Kp, float Ki, float Kd, float Kf);
void pidInitCopy (PID pid, PID toCopy);
float pidCalculate (PID pid, float setPoint, float processVariable);

#endif
