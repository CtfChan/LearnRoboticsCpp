#include "common.hpp"


float deg2rad(float deg) {
    return deg * M_PI/ 180.0f;
}

// clip angle to [-pi, pi]
float normalizeAngle(float angle) {
    while (angle > M_PI)
        angle-= 2.0f * M_PI;
    while (angle < -M_PI) 
        angle += 2.0f*M_PI;
    return angle;
}

std::vector<std::tuple<float, float, float, float>> 
    poseToVector(float x, float y, float theta, float r) {
    float dx = r * std::cos(theta);
    float dy = r * std::sin(theta);
    auto arrow = std::make_tuple(x, y, dx ,dy);
    return { arrow };
}


