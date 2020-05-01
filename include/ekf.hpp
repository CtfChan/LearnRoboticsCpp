#pragma once

#include "gnuplot-iostream.h"

#include<iostream>
#include<random>
#include<cmath>

#include<Eigen/Eigen>

using Path = std::vector<std::pair<float, float>>;


float deg2rad(float deg) {
    return (deg / 180.f * M_PI);
}


Eigen::Vector4f motionModel(const Eigen::Vector4f& x, const Eigen::Vector2f& u, const float dt);

Eigen::Vector2f observationModel(const Eigen::Vector4f& x);

void extendedKalmanFilter(Eigen::Vector4f& x_est, Eigen::Matrix4f& P_est,
                       const Eigen::Vector2f& u, const Eigen::Vector2f& z, 
                       const Eigen::Matrix4f& Q, const Eigen::Matrix2f& R,
                       const float dt );

// jacobian of motion
Eigen::Matrix4f jacobianF(const Eigen::Vector4f& x, const Eigen::Vector2f& u, const float dt );

// jacobian of observation
Eigen::Matrix<float, 2, 4> jacobianH();