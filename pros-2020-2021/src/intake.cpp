#include "intake.h"
#include "main.h"
#include <cmath>
#include <memory>

namespace intake
{
    // intake motors
    okapi::Motor left(6), right(-10);

    // buttons for opcontrol
    okapi::ControllerButton intakePositiveButton(okapi::ControllerDigital::L1);
	okapi::ControllerButton intakeNegativeButton(okapi::ControllerDigital::L2);
    okapi::ControllerButton strafeVerticalButton(okapi::ControllerDigital::B);

    // Variables for auton
    std::unique_ptr<ADIAnalogIn> intakeSensor;
    std::unique_ptr<okapi::MedianFilter<10>> filter;
    std::unique_ptr<Task> intakeTaskHandler;
    std::unique_ptr<Mutex> intakeMutex;
    static IntakeState state = OFF;
    static int targetNumber = 0;
    static bool initialized = false;
    static constexpr int detectionThreshold = -500;

    void intakeTask(void *ign)
    {
        IntakeState currentState = OFF;
        int currentTarget = 0;

        while (true)
        {
            intakeMutex->take(10);
            currentState = state;
            currentTarget = targetNumber;
            intakeMutex->give();

            switch (currentState)
            {
                case OFF:
                    moveVoltage(0);
                    break;
                case INTAKE:
                    moveVoltage(12000);
                    break;
                case OUTTAKE:
                    moveVoltage(-12000);
                    break;
                case NUMBER_IN:
                    while (currentTarget > 0)
                    {
                        intakeMutex->take(10);
                        currentState = state;
                        intakeMutex->give();
                        if (currentState != NUMBER_IN)
                        {
                            moveVoltage(0);
                            break;
                        }
                        
                        // Intakes until sensor is below threshold
                        moveVoltage(12000);
                        indexer::moveVoltageSafe(12000);
                        if (readFilterSensor() < detectionThreshold)
                        {
                            // Decreases currentTarget each time sensor is below threshold
                            currentTarget--;

                            // Break early if it is the last ball to be intaked
                            if (currentTarget == 0)
                                break;
                            
                            // Loop until ball is past the sensor so a ball isn't double counted
                            while (readFilterSensor() < detectionThreshold)
                            {
                                delay(20);
                            }
                        }
                        delay(20);
                    }
                    moveVoltage(0);
                    indexer::moveVoltageSafe(0);
                    intakeMutex->take(10);
                    state = OFF;
                    targetNumber = 0;
                    intakeMutex->give();
                    break;
                case NUMBER_OUT:
                    while (currentTarget > 0)
                    {
                        intakeMutex->take(10);
                        currentState = state;
                        intakeMutex->give();
                        if (currentState != NUMBER_OUT)
                        {
                            moveVoltage(0);
                            break;
                        }
                        
                        // Intakes until sensor is below threshold
                        moveVoltage(-12000);
                        indexer::moveVoltageSafe(-12000);
                        if (readFilterSensor() < detectionThreshold)
                        {
                            // Decreases currentTarget each time sensor is below threshold
                            currentTarget--;
                            
                            // Loop until ball is past the sensor so a ball isn't double counted
                            while (readFilterSensor() < detectionThreshold)
                            {
                                delay(20);
                            }
                        }
                        delay(20);
                    }
                    moveVoltage(0);
                    indexer::moveVoltageSafe(0);
                    intakeMutex->take(10);
                    state = OFF;
                    targetNumber = 0;
                    intakeMutex->give();
                    break;
            }
            delay(20);
        }
    }
    void init()
    {
        if (!initialized)
        {
            intakeSensor = std::make_unique<ADIAnalogIn>('H');
            delay(200);
            intakeSensor->calibrate();
            printf("Intake Sensor Calibrated: %d\tRaw: %d\n", intakeSensor->get_value_calibrated(), intakeSensor->get_value());

            intakeMutex = std::make_unique<Mutex>();
            filter = std::make_unique<okapi::MedianFilter<10>>();
            delay(20);
            
            intakeTaskHandler = std::make_unique<Task>(intakeTask, nullptr, TASK_PRIORITY_DEFAULT);
            initialized = true;
        }
        else
        {
            intakeTaskHandler->resume();
        }
    }
    void stop()
    {
        if (intakeTaskHandler)
            intakeTaskHandler->suspend();
    }
    int waitUntilStopped()
    {
        return waitUntilStopped(60000);
    }
    int waitUntilStopped(int timeout)
    {
        int timer = millis();
        while (getState() != OFF)
        {
            if (millis() - timer > timeout)
            {
                setState(OFF);
                return -1;
            }
            delay(50);
        }
        return 0;
    }
    IntakeState getState()
    {
        return state;
    }
    int getTargetNumber()
    {
        return targetNumber;
    }
    void setState(IntakeState newState, int newTarget)
    {
        intakeMutex->take(10);
        state = newState;
        targetNumber = newTarget;
        intakeMutex->give();
    }
    void setState(IntakeState newState)
    {
        setState(newState, 0);
    }
    int readFilterSensor()
    {
        for (int i = 0; i < 10; i++)
        {
            filter->filter(intakeSensor->get_value_calibrated()); 
            delay(5);  
        }
        return filter->getOutput();
    }
    void moveVoltage(int intakeVolt)
    {
        left.moveVoltage(intakeVolt);
		right.moveVoltage(intakeVolt);
    }
    void opcontrol()
    {
        int intakeVolt = 0;

        if (intakePositiveButton.isPressed())
			intakeVolt = 12000;
        else if (intakeNegativeButton.isPressed())
			intakeVolt = -12000;

        if (strafeVerticalButton.isPressed())
            intakeVolt = 12000 * ((double) master.get_analog(ANALOG_LEFT_Y) / 127);

        

        moveVoltage(intakeVolt);

        //printf("Intake Sensor Filtered: %d\tCalibrated: %d\tRaw:%d\n", readFilterSensor(), std::nan, std::nan);
    }
}