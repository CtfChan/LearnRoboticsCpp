#pragma once

#include <Eigen/Eigen>
#include <random>

// useful for plotting 
using Path = std::vector<std::pair<float, float>>;
using Ellipse =  std::vector<std::pair<float, float>>;


// shared
Eigen::Vector4f motionModel(const Eigen::Vector4f& x, const Eigen::Vector2f& u, const float dt);

Eigen::Vector2f observationModel(const Eigen::Vector4f& x);

Ellipse generateEllipse(const Eigen::Vector4f& x_est, const Eigen::Matrix4f& P_est);

