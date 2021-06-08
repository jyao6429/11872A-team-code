#include "main.h"
#include <memory>

namespace odom
{
    // Current pose of the robot
    pose robotPose;

    // Encoders
    std::unique_ptr<pros::ADIEncoder> leftEncoder;
    std::unique_ptr<pros::ADIEncoder> rightEncoder;
    std::unique_ptr<pros::ADIEncoder> backEncoder;


    // Variables for odomTask
    std::unique_ptr<pros::Task> odomTaskHandler;
    std::unique_ptr<pros::Mutex> poseMutex;
    static bool initialized = false;
    static bool isLogging = false;

    // Constant physical variables of the robot
    static constexpr double trackWidth = 15.0328;
    static constexpr double backLength = trackWidth / 2;
    static constexpr double wheelDiameter = 2.742;
    static constexpr double encoderResolution = 360;

    void odomTask(void *ign)
    {
        int prevLeft = 0, prevRight = 0, prevBack = 0;
        uint32_t now = millis();

        while (true)
        {
            poseMutex->take(1);

            int currentLeft = leftEncoder->get_value();
            int currentRight = rightEncoder->get_value();
            int currentBack = backEncoder->get_value();
            
            // Calculate the distance travelled by each tracking wheel fron the previous step
            double deltaLeftDistance = (double) (currentLeft - prevLeft) * okapi::pi * wheelDiameter / (double) encoderResolution;
            double deltaRightDistance = (double) (currentRight - prevRight) * okapi::pi * wheelDiameter / (double) encoderResolution;
            double deltaBackDistance = (double) (currentBack - prevBack) * okapi::pi * wheelDiameter / (double) encoderResolution;

            // Calculate the change in robot orientation
            double deltaAngle = (deltaLeftDistance - deltaRightDistance) / (trackWidth);

            // Calculate the local offset of the robot
            double localX, localY;
            if (deltaAngle == 0.0)
            {
                localX = deltaBackDistance;
                localY = deltaRightDistance;
            }
            else
            {
                localX = 2.0 * std::sin(deltaAngle / 2.0) * ((deltaBackDistance / deltaAngle) + backLength);
                localY = 2.0 * std::sin(deltaAngle / 2.0) * ((deltaRightDistance / deltaAngle) + trackWidth / 2.0);
            }

            // Calculate variables for linear transformation
            double avgAngle = robotPose.theta + (deltaAngle / 2);
            double cosAvg = std::cos(avgAngle);
            double sinAvg = std::sin(avgAngle);

            // Update global position of robot with the rotated local offset vector
            robotPose.x += localX * cosAvg + localY * sinAvg;
            robotPose.y += localX * -sinAvg + localY * cosAvg;
            robotPose.theta += deltaAngle;
            poseMutex->give();

            if (isLogging)
            {
                printf("X: %3.3f\tY: %3.3f\tT: %3.3f\n", robotPose.x, robotPose.y, robotPose.theta);
            }

            // Update previous values
            prevLeft = currentLeft;
            prevRight = currentRight;
            prevBack = currentBack;
            Task::delay_until(&now, 5);
        }
    }
    void start(bool log)
    {
        if (!initialized)
        {
            leftEncoder = std::make_unique<ADIEncoder>(1, 2, false);
            rightEncoder = std::make_unique<ADIEncoder>(3, 4, false);
            backEncoder = std::make_unique<ADIEncoder>(5, 6, true);
            poseMutex = std::make_unique<Mutex>();
            isLogging = log;
            delay(20);

            odomTaskHandler = std::make_unique<Task>(odomTask, nullptr, TASK_PRIORITY_DEFAULT + 2);
            initialized = true;
        }
        else
        {
            odomTaskHandler->resume();
        }
    }
    void stop()
    {
        if (odomTaskHandler)
            odomTaskHandler->suspend();
    }
    pose getPose()
    {
        return robotPose;
    }
    void setPose(pose newPose)
    {
        poseMutex->take(10);
        robotPose = newPose;
        poseMutex->give();
    }

}