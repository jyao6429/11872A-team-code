#ifndef CHASSIS_H
#define CHASSIS_H

#include "okapi/api.hpp"

namespace chassis
{
    // States for state machine during auton
    typedef enum ChassisState
    {
        OFF,
        SKIP,
        MTT
    } ChassisState;
        
    // Container for variables needed in async moveToTarget function
    typedef struct MTTContainer
    {
        okapi::QLength x, y;
        okapi::QAngle theta;
        double maxSpeed, maxOmega;
        bool park;

    } MTTContainer;
    
    /** 
     * Initialize chassisTask during startup or resume task after suspension
     */
    void init();
    /** 
     * Suspends chassisTask
     */
    void stop();
    /** 
     * Sets the chassis state machine to given value
     *
     * @param newState - the new state for the chassis
     */
    void setState(ChassisState newState);
    /** 
     * Gets the current state of the chassis state machine
     *
     * @return the current state
     */
    ChassisState getState();
    /** 
     * Waits until the chassis state is no longer in MTT or until the timeout
     *
     * @param timeout - the timeout desired in milliseconds
     * @return 0 if settled, -1 if timed out
     */
    int waitUntilSettled(int timeout);
    /** 
     * Waits until the chassis state is no longer in MTT
     *
     * @return 0 once settled
     */
    int waitUntilSettled();
    /** 
     * Waits until the chassis state is no longer in MTT, until timeout, or is stuck
     *
     * @param timeout - the timeout desired in milliseconds
     * @return 0 if settled, -1 if timed out, -2 if stuck
     */
    int waitUntilStuck(int timeout);
    /** 
     * Returns the velocity components of the robot in units / second after 50 ms
     *
     * @return the current velocity of the robot
     */
    okapi::OdomState getVelocity();
    /** 
     * Moves robot to desired position and orientation on field without blocking
     *
     * @param targetX - target X coordinate on the field
     * @param targetY - target Y coordinate on the field
     * @param targetTheta - target orientation of the robot
     * @param maxSpeed - [0 - 1] maximum linear speed of the movement
     * @param maxOmega - [0 - 1] maximum angular velocity of the movement     
     * @param park - true if the robot should settle at target before exiting   
     */
    void moveToTargetAsync(okapi::QLength targetX, okapi::QLength targetY, okapi::QAngle targetTheta, double maxSpeed, double maxOmega, bool park);
    /** 
     * Moves robot to desired position and orientation on field while blocking
     *
     * @param targetX - target X coordinate on the field
     * @param targetY - target Y coordinate on the field
     * @param targetTheta - target orientation of the robot
     * @param maxSpeed - [0 - 1] maximum linear speed of the movement
     * @param maxOmega - [0 - 1] maximum angular velocity of the movement     
     * @param park - true if the robot should settle at target before exiting   
     */
    void moveToTarget(okapi::QLength targetX, okapi::QLength targetY, okapi::QAngle targetTheta, double maxSpeed, double maxOmega, bool park);
    /**
    * Calculates nearest equivalent angle in radians for shortest angle to turn
    *
    * @param angle - the target orientation in radians
    * @param reference - the current orientation of the robot
    *
    * @return the target orientation + 2 * pi * k added
    */
    double nearestEquivalentAngle(double angle, double reference);
    /**
    * Sets the current robot pose
    *
    * @param newX - the desired x coordinate of the new state
    * @param newY - the desired y coordinate of the new state
    * @param newTheta - the desired orientation of the new state
    */
    void setOdomState(okapi::QLength newX, okapi::QLength newY, okapi::QAngle newTheta);
    /**
    * Resets the robot pose to (0, 0, 0)
    */
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
    /**
    * Fucntion for chassis control during driver
    */
    void opcontrol();
}

#endif