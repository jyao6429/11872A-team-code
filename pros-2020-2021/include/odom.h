#ifndef ODOM_H
#define ODOM_H

namespace odom
{
    // Class for the coordinates and orientation of the robot in inches and radians
    class pose
    {
        public:
        double x, y, theta;
        pose() {}
        pose(const pose& p2)
        {
            x = p2.x;
            y = p2.y;
            theta = p2.theta;
        }
    };

    /** 
     * Initialize odomTask during startup or resume task after suspension
     */
    void start(bool log = false);
    /** 
     * Suspends odomTask
     */
    void stop();
    /** 
     * Returns the current pose of the robot
     *
     * @return the current pose
     */
    pose getPose();
    /** 
     * Sets the current pose of the robot
     *
     * @param newPose - the desired pose of the robot
     */
    void setPose(pose newPose);
}

#endif