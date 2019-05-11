#include "main.h"
#include "constants.h"
#include "chassis.h"

// Physical parameters in inches
const double sL{0.0};   // distance from center to left tracking wheel
const double sR{0.0};   // distance from center to right tracking wheel
const double sB{0.0};   // distance from center to back tracking wheel

// Previous positions
double prevX{0.0};
double prevY{0.0};
double prevAngle{0.0};
double resetAngle{0.0};

// Not sure if using v5 or cortex, just rewrite these functions to make code work
int getLeftEncoder() {return 0;}
int getRightEncoder() {return 0;}
int getBackEncoder() {return 0;}
