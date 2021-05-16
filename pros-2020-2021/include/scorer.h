#ifndef SCORER_H
#define SCORER_H

namespace scorer
{
    typedef enum ScorerState
    {
        OFF,
        SCORE,
        EJECT,
        NUMBER_SCORE,
    } ScorerState;
    void init();
    void stop();
    int waitUntilStopped();
    int waitUntilStopped(int timeout);
    ScorerState getState();
    void setState(ScorerState newState, int newTarget);
    void setState(ScorerState newState);
    int readFilterSensor();
    void opcontrol();
}

#endif