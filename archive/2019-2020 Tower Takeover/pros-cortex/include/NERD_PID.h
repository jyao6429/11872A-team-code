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

/**
 * initialize pid structure, set parameters
 *
 * pid - instance of PID structure
 * Kp - proportional gain
 * Ki - integral gain
 * Kd - derivative gain
 */
void pidInit(PID *pid, double Kp, double Ki, double Kd, double Kf);
/**
 * initialize pid structure, set parameters based on another PID structure
 *
 * @param pid - instance of PID structure
 * @param toCopy - PID instance to copy settings from
 */
void pidInitCopy(PID *pid, PID *toCopy);
/**
 * calculate pid output
 *
 * @param pid - instance of PID structure
 * @param setPoint - set point of PID controller
 * @param processVariable - sensor/feedback value
 *
 * @return output value of the control loop
 */
double pidCalculate(PID *pid, double setPoint, double processVariable);

#endif
