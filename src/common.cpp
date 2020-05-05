#include "common.hpp"

#include <algorithm>

#include <Eigen/Eigen>

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

std::array<float, 3> quadraticCoefficients(std::array<float, 3>& x, std::array<float, 3>& y) {
    Eigen::Matrix3f A;
    Eigen::Vector3f b;
    A << std::pow(x[0], 2), x[0], 1,
        std::pow(x[1], 2), x[1], 1,
        std::pow(x[2], 2), x[2], 1;
    b << y[0], y[1], y[2];

    Eigen::Vector3f coeffs = A.inverse() * b;
    
    return {coeffs[0] , coeffs[1], coeffs[2]};
}

float quadraticInterpolation(std::array<float, 3>& coeff, float x) {
    return coeff[0] * std::pow(x, 2) + coeff[1] * x + coeff[2];
}





Arrow poseToVector(float x, float y, float theta, float r) {
    float dx = r * std::cos(theta);
    float dy = r * std::sin(theta);
    auto arrow = std::make_tuple(x, y, dx ,dy);
    return { arrow };
}




Arrow trajectoryToVector(Pose2DTrajectory& traj) {
    Arrow arr;

    size_t n = traj.x.size();
    for (size_t i = 0; i < n-1; ++i) {
        float x = traj.x[i];
        float y = traj.y[i];
        float dx = traj.x[i+1] - x;
        float dy = traj.y[i+1] - y;
        arr.emplace_back(x, y, dx, dy);
    }

    return arr;
}
