#ifndef INTAKE_H
#define INTAKE_H

namespace intake
{
    typedef enum IntakeState
    {
        OFF,
        INTAKE,
        OUTTAKE,
        NUMBER_IN,
        NUMBER_OUT
    } IntakeState;
    void init();
    void stop();
    void waitUntilStopped();
    IntakeState getState();
    void setState(IntakeState newState, int newTarget);
    void setState(IntakeState newState);
    void moveVoltage(int intakeVolt);
    int readFilterSensor();
    void opcontrol();
}

#endif