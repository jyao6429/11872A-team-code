#ifndef NERD_PID_H
#define NERD_PID_H

/**
 * PID controller data structure
 */
typedef struct
{
	double Kp;
	double Ki;
	double Kd;
	double Kf;
	double sigma;
	double lastValue;
	unsigned long lastTime;
	double lastSetPoint;
} PID;

void pidInit (PID pid, double Kp, double Ki, double Kd);
void pidInitCopy (PID pid, PID toCopy);
double pidCalculate (PID pid, double setPoint, double processVariable);

#endif
