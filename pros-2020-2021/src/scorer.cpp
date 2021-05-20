#include "main.h"

namespace scorer
{
    okapi::Motor scorer(1);

    okapi::ControllerButton scorerPositiveButton(okapi::ControllerDigital::R2);
	okapi::ControllerButton scorerNegativeButton(okapi::ControllerDigital::R1);

    std::unique_ptr<ADIAnalogIn> scorerSensor;
    std::unique_ptr<okapi::MedianFilter<10>> filter;
    std::unique_ptr<Task> scorerTaskHandler;
    std::unique_ptr<Mutex> scorerMutex;
    static ScorerState state = OFF;
    static int targetNumber = 0;
    static bool initialized = false;

    static constexpr int detectionThreshold = -500;

    void scorerTask(void *ign)
    {
        ScorerState currentState = OFF;
        int currentTarget = 0;

        while (true)
        {
            scorerMutex->take(10);
            currentState = state;
            currentTarget = targetNumber;
            scorerMutex->give();

            switch (currentState)
            {
                case OFF:
                    scorer.moveVoltage(0);
                    break;
                case SCORE:
                    scorer.moveVoltage(12000);
                    break;
                case EJECT:
                    scorer.moveVoltage(-12000);
                    break;
                case NUMBER_SCORE:
                    while (currentTarget > 0)
                    {
                        scorerMutex->take(10);
                        currentState = state;
                        scorerMutex->give();
                        if (currentState != NUMBER_SCORE)
                        {
                            scorer.moveVoltage(0);
                            break;
                        }
                        
                        scorer.moveVoltage(12000);
                        indexer::moveVoltageSafe(12000);
                        if (readFilterSensor() < detectionThreshold)
                        {
                            currentTarget--;
                            while (readFilterSensor() < detectionThreshold)
                            {
                                delay(20);
                            }
                        }
                        delay(20);
                    }
                    scorer.moveVoltage(0);
                    indexer::moveVoltageSafe(0);
                    scorerMutex->take(10);
                    state = OFF;
                    targetNumber = 0;
                    scorerMutex->give();
                    break;
            }
            delay(20);
        }
    }
    void init()
    {
        if (!initialized)
        {
            scorerSensor = std::make_unique<ADIAnalogIn>('G');
            delay(200);
            scorerSensor->calibrate();
            printf("Scorer Sensor Calibrated: %d\tRaw: %d\n", scorerSensor->get_value_calibrated(), scorerSensor->get_value());

            scorerMutex = std::make_unique<Mutex>();
            filter = std::make_unique<okapi::MedianFilter<10>>();
            delay(20);
            
            scorerTaskHandler = std::make_unique<Task>(scorerTask, nullptr, TASK_PRIORITY_DEFAULT);
            initialized = true;
        }
        else
        {
            scorerTaskHandler->resume();
        }
    }
    void stop()
    {
        if (scorerTaskHandler)
            scorerTaskHandler->suspend();
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
                return -1;
            delay(50);
        }
        return 0;
    }
    ScorerState getState()
    {
        return state;
    }
    int getTargetNumber()
    {
        return targetNumber;
    }
    void setState(ScorerState newState, int newTarget)
    {
        scorerMutex->take(10);
        state = newState;
        targetNumber = newTarget;
        scorerMutex->give();
    }
    void setState(ScorerState newState)
    {
        setState(newState, 0);
    }
    int readFilterSensor()
    {
        for (int i = 0; i < 10; i++)
        {
            filter->filter(scorerSensor->get_value_calibrated()); 
            delay(5);  
        }
        return filter->getOutput();
    }
    void opcontrol()
    {
        int scorerVolt = 0;

        if (scorerNegativeButton.isPressed())
			scorerVolt = -12000;
		else if (scorerPositiveButton.isPressed())
			scorerVolt = 12000;

        scorer.moveVoltage(scorerVolt);
    }
}