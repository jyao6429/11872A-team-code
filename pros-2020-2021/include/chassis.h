#ifndef CHASSIS_H
#define CHASSIS_H

namespace chassis
{
    /** 
     * strafe chassis along target vector and angular velocity
     *
     * @param theta target angle to move in radians
     * @param omega [-1 - 1] target angular velocity
     * @param speed [0 - 1] target translational speed
     */
    void moveVector(double theta, double omega, double speed);
    void opcontrol();
}

#endif