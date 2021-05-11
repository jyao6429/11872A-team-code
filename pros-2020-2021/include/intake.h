#ifndef INTAKE_H
#define INTAKE_H

namespace intake
{
    enum IntakeState
    {
        OFF,
        INTAKE,
        OUTTAKE,
        NUMBER_IN,
        NUMBER_OUT
    };
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