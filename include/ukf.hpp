#pragma once

#include "gnuplot-iostream.h"

#include <iostream>
#include <random>
#include <cmath>

#include <Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>


using Path = std::vector<std::pair<float, float>>;
using Ellipse =  std::vector<std::pair<float, float>>;

float deg2rad(float deg) {
    return (deg / 180.f * M_PI);
}


Eigen::Vector4f motionModel(const Eigen::Vector4f& x, const Eigen::Vector2f& u, const float dt);

Eigen::Vector2f observationModel(const Eigen::Vector4f& x);

// generates (wm, wc, gamma)
std::tuple<std::vector<float>, std::vector<float>, float> 
    setupWeights(float nx, float alpha, float kappa, float beta);

void unscentedKalmanFilter(Eigen::Vector4f& x_est, Eigen::Matrix4f& P_est,
                       const Eigen::Vector2f& u, const Eigen::Vector2f& z, 
                       const Eigen::Matrix4f& Q, const Eigen::Matrix2f& R,
                       const float dt, const float gamma,
                       const std::vector<float>& wm, 
                       const std::vector<float>& wc);

 std::vector<Eigen::Vector4f> generateSigmaPoints(
     const Eigen::Vector4f& x_est, const Eigen::Matrix4f& P_est, const float gamma);


Ellipse generateEllipse(const Eigen::Vector4f& x_est, const Eigen::Matrix4f& P_est);

