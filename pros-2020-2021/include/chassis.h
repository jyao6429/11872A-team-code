#ifndef CHASSIS_H
#define CHASSIS_H

#include "main.h"

namespace chassis
{
    void moveToTarget(okapi::QLength targetX, okapi::QLength targetY, okapi::QAngle targetTheta, double maxSpeed, double maxOmega, bool park);
    double nearestEquivalentAngle(double angle, double reference);
    void setOdomState(okapi::QLength newX, okapi::QLength newY, okapi::QAngle newTheta);
    void resetOdom();
    /** 
     * strafe chassis along target vector and angular velocity
     *
     * @param theta - target angle to move in radians
     * @param omega - [-1 - 1] target angular velocity
     * @param speed - [0 - 1] target translational speed
     */
    void strafeVector(double theta, double omega, double speed);
    /** 
     * move chassis along target vector and angular velocity, with the ability to turn in place
     *
     * @param theta - target angle to move in radians
     * @param omega - [-1 - 1] target angular velocity
     * @param speed - [0 - 1] target translational speed
     */
    void moveVector(double theta, double omega, double speed);
    void opcontrol();
}

#endif