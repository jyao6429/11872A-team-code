#ifndef APS_H
#define APS_H

double prevPos[];
double prevAngle;
double resetAngle;

int getLeftEncoder();
int getRightEncoder();
int getBackEncoder();

void startTracking();

double nearestEquivalentAngle(double ref, double target);
void convertPolar(double *source, double *target);
void convertCart(double *source, double *target);
double encoderToRad(int count, int ticksPerRotation);
double degToRad(double degrees);
double radToDeg(double rads);

#endif
