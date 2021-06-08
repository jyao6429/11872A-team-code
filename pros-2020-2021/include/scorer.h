#ifndef SCORER_H
#define SCORER_H

namespace scorer
{
    // States for state machine during auton
    typedef enum ScorerState
    {
        OFF,
        SCORE,
        EJECT,
        NUMBER_SCORE,
    } ScorerState;

    /** 
     * Initialize scorerTask during startup or resume task after suspension
     */
    void init();
    /** 
     * Suspends scorerTask
     */
    void stop();
    /** 
     * Waits until the scorer state is OFF
     *
     * @return 0 once settled
     */
    int waitUntilStopped();
    /** 
     * Waits until the scorer state is OFF or until a timeout
     *
     * @param timeout - the timeout desired in milliseconds
     * @return 0 if settled, -1 if timed out
     */
    int waitUntilStopped(int timeout);
    /** 
     * Returns the current state of the scorer state machine
     *
     * @return the current scorer state
     */
    ScorerState getState();
    /** 
     * Sets the desired state of the scorer state machine with a target number of balls to score
     *
     * @param newState - the desired state
     * @param newTarget - the number of balls to score
     */
    void setState(ScorerState newState, int newTarget);
    /** 
     * Sets the desired state of the scorer state machine and the target number of balls to 0
     *
     * @param newState - the desired state
     */
    void setState(ScorerState newState);
    /** 
     * Gives the median of ten measurements of the scorer line sensor over 50 ms
     *
     * @return the filtered reading
     */
    int readFilterSensor();
    /**
    * Fucntion for scorer control during driver
    */
    void opcontrol();
}

#endif