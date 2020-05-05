#include "common.hpp"
#include <algorithm>


void BicycleModelRobot::update(float acc, float delta, float dt) {
    delta = std::clamp(delta, -max_steer, max_steer);
    
    x += v * std::cos(yaw) * dt;
    y += v * std::sin(yaw) * dt;
    yaw += v / L * std::tan(delta) * dt;
    yaw = normalizeAngle(yaw);
    v += acc * dt;
}



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




Arrow poseToVector(float x, float y, float theta, float r) {
    float dx = r * std::cos(theta);
    float dy = r * std::sin(theta);
    auto arrow = std::make_tuple(x, y, dx ,dy);
    return { arrow };
}


