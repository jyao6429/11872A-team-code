#include "main.h"

#define DELTA_T 0.02
#define V_MAX 36.0
#define A_MAX 24.0
#define J_MAX 144.0

void testPathFollow(void *ignore)
{
printf("Creating Waypoints\n");
  int POINT_LENGTH = 3;

  Waypoint *points = (Waypoint*)malloc(sizeof(Waypoint) * POINT_LENGTH);

  Waypoint p1 = { 0, 0, 0 };      // Waypoint @ x=-4, y=-1, exit angle=45 degrees
  Waypoint p2 = { -12, 24, d2r(-90) };             // Waypoint @ x=-1, y= 2, exit angle= 0 radians
  Waypoint p3 = {  -12, 48, 0 };             // Waypoint @ x= 2, y= 4, exit angle= 0 radians
  points[0] = p1;
  points[1] = p2;
  points[2] = p3;

printf("Created Waypoints, now preparing trajectory\n");

  TrajectoryCandidate candidate;
  pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_FAST, DELTA_T, V_MAX, A_MAX, J_MAX, &candidate);

printf("Prepared trajectory, now freeing points in memory\n");
  free(points);

  int length = candidate.length;

printf("Attempting to allocate memory for trajectory\n");
  Segment *trajectory = malloc(length * sizeof(Segment));
printf("Allocated memory, now generating trajectory\n");

  pathfinder_generate(&candidate, trajectory);

printf("Generated trajectory, attempting to allocate memory for left and right trajectories\n");

  Segment *leftTrajectory = (Segment*)malloc(sizeof(Segment) * length);
  Segment *rightTrajectory = (Segment*)malloc(sizeof(Segment) * length);

printf("Allocated memory, now modifying trajectory for tank\n");

  double wheelbase_width = sL + sR;

  pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, wheelbase_width);

  printf("Starting drive\n");
  // Do something with the trajectories...
  FollowerConfig config = {0.0, 0.0, 0.0, 127 / V_MAX, 0.0};
  DistanceFollower leftFollower = {0.0, robotPose[POSE_ANGLE], 0.0, 0, 0};
  DistanceFollower rightFollower = {0.0, robotPose[POSE_ANGLE], 0.0, 0, 0};

  for (int i = 0; i < length; i++)
  {
    double leftDistance = 0.0;
    double rightDistance = 0.0;

    if (leftFollower.segment > 0)
    {
      Segment lastLeft = leftTrajectory[leftFollower.segment - 1];
      Segment lastRight = rightTrajectory[rightFollower.segment - 1];

      leftDistance = sqrt(pow(leftWheelPosition[X_COMP] - lastLeft.x, 2) + pow(leftWheelPosition[Y_COMP] - lastLeft.y, 2));
      rightDistance = sqrt(pow(rightWheelPosition[X_COMP] - lastRight.x, 2) + pow(rightWheelPosition[Y_COMP] - lastRight.y, 2));
    }

    int leftPower = pathfinder_follow_distance(config, &leftFollower, leftTrajectory, length, leftDistance);
    int rightPower = pathfinder_follow_distance(config, &rightFollower, rightTrajectory, length, rightDistance);

    powerMotors(leftPower, rightPower);

    delay(20);
  }

  free(trajectory);
  free(leftTrajectory);
  free(rightTrajectory);
}
