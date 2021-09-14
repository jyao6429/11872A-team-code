#ifndef INTAKE_H
#define INTAKE_H

namespace intake
{
    // States for state machine during auton
    typedef enum IntakeState
    {
        OFF,
        INTAKE,
        OUTTAKE,
        NUMBER_IN,
        NUMBER_OUT
    } IntakeState;

    /** 
     * Initialize intakeTask during startup or resume task after suspension
     */
    void init();
    /** 
     * Suspends intakeTask
     */
    void stop();
    /** 
     * Waits until the intake state is OFF
     *
     * @return 0 once settled
     */
    int waitUntilStopped();
    /** 
     * Waits until the intake state is OFF or until a timeout
     *
     * @param timeout - the timeout desired in milliseconds
     * @return 0 if settled, -1 if timed out
     */
    int waitUntilStopped(int timeout);
    /** 
     * Returns the current state of the intake state machine
     *
     * @return the current intake state
     */
    IntakeState getState();
    /** 
     * Sets the desired state of the intake state machine with a target number of balls to intake/outtake
     *
     * @param newState - the desired state
     * @param newTarget - the number of balls to intake/outtake
     */
    void setState(IntakeState newState, int newTarget);
    /** 
     * Sets the desired state of the intake state machine and the target number of balls to 0
     *
     * @param newState - the desired state
     */
    void setState(IntakeState newState);
    /** 
     * Moves indexer with target voltage
     *
     * @param intakeVolt - desired voltage for the indexer motor
     */
    void moveVoltage(int intakeVolt);
    /** 
     * Gives the median of ten measurements of the intake line sensor over 50 ms
     *
     * @return the filtered reading
     */
    int readFilterSensor();
    /**
    * Fucntion for intake control during driver
    */
    void opcontrol();
}

#endif