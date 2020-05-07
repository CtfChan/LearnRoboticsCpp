#pragma once

#include "localization/filters.hpp"

#include <unsupported/Eigen/MatrixFunctions> // for ukf


// UKF
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





