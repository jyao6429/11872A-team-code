#ifndef ODOM_H
#define ODOM_H

namespace odom
{
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

    void start(bool log = false);
    void stop();
    pose getPose();
    void setPose(pose newPose);
}

#endif