#include "localization/filters.hpp"

#include <iostream>

// shared functionality by most filters

// x is state, u is control
Eigen::Vector4f motionModel(const Eigen::Vector4f& x, const Eigen::Vector2f& u, const float dt) {
    const static Eigen::Matrix4f F = (Eigen::Matrix4f() << 
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0
    ).finished();

    Eigen::Matrix<float, 4, 2> B;
    B << dt * std::cos(x(2)),  0,
         dt * std::sin(x(2)),  0,
                        0.0,  dt,
                        1.0,  0.0;

    return F * x + B * u;
}

// depends only on state
Eigen::Vector2f observationModel(const Eigen::Vector4f& x) {
    Eigen::Matrix<float, 2, 4> H;
    H << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0;
    return H * x;
}


Ellipse generateEllipse(const Eigen::Vector4f& x_est, const Eigen::Matrix4f& P_est) {
    Ellipse ep;

    Eigen::Matrix2f p_xy = P_est.block(0, 0, 2, 2); // get cov of x,y
    Eigen::EigenSolver<Eigen::Matrix2f> ces(p_xy);
    Eigen::Matrix2f evals = ces.pseudoEigenvalueMatrix();
    Eigen::Matrix2f evecs = ces.pseudoEigenvectors(); // row major
 
    // biger eval is 0th idx
    float a = std::sqrt(evals(0,0));
    float b = std::sqrt(evals(1,1));

    // use angle of dominant direction w/ x-axis to determine rotation
    float angle = std::atan2(evecs(0, 1), evecs(0, 0));
    Eigen::Matrix2f rot; // rotation matrix
    rot << std::cos(angle), std::sin(angle),
           -std::sin(angle), std::cos(angle);


    for (float t = 0.0; t < 2 * M_PI + 0.1; t += 0.1) {
        Eigen::Vector2f x;
        x << a * std::cos(t), b * std::sin(t);
        Eigen::Vector2f fx = rot * x;
        ep.emplace_back(fx(0) + x_est(0), fx(1) + x_est(1));
    }

    return ep;
}










