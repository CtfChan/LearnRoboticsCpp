#pragma once

#include <math.h>
#include <tuple>
#include <vector>


using Path = std::vector<std::pair<float, float>>;
using Arrow = std::vector<std::tuple<float, float, float, float>>;

struct Pose2D {
    float x;
    float y;
    float theta;
};

struct Pose2DTrajectory {
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> theta;
};

struct BicycleModelRobot {
    float x;
    float y;
    float yaw;
    float v;
    float L; // wheelbase
    float max_steer;

    // update state acceleration, steering, dt
    void update(float acc, float delta, float dt);
};

// Turn degrees to radians
float deg2rad(float deg);

// clip angle to [-pi, pi]
float normalizeAngle(float angle);

// for visualization
Arrow poseToVector(float x, float y, float theta, float r=0.5f);
