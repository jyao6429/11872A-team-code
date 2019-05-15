#ifndef APS_H
#define APS_H

double prevPos[];
double prevAngle;
double resetAngle;

int getLeftEncoder();
int getRightEncoder();
int getBackEncoder();

void initializeAPS(double startX, double startY, double startAngle);
void startTracking(void *ignore);

double nearestEquivalentAngle(double ref, double target);
void convertPolar(double *source, double *target);
void convertCart(double *source, double *target);
double encoderToRad(int count, int ticksPerRotation);
double degToRad(double degrees);
double radToDeg(double rads);

#endif
