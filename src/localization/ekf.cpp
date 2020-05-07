#include "localization/ekf.hpp"
#include "localization/filters.hpp"

Eigen::Matrix4f jacobianF(const Eigen::Vector4f& x, const Eigen::Vector2f& u, const float dt ) {
    Eigen::Matrix4f J_F = Eigen::Matrix4f::Identity();
    float v = u(0);
    float phi = x(2);
    J_F(0, 2) = -v * std::sin(phi) * dt;
    J_F(0, 3) = std::cos(phi) * dt;
    J_F(1, 2) = v * std::cos(phi) * dt;
    J_F(1, 3) = std::sin(phi) * dt;

    return J_F;
}

Eigen::Matrix<float, 2, 4> jacobianH() {
    Eigen::Matrix<float, 2, 4> J_H;
    J_H << 1.0, 0.0, 0.0, 0.0,
           0.0, 1.0, 0.0, 0.0;
    return J_H;
}



void extendedKalmanFilter(Eigen::Vector4f& x_est, Eigen::Matrix4f& P_est,
                       const Eigen::Vector2f& u, const Eigen::Vector2f& z, 
                       const Eigen::Matrix4f& Q, const Eigen::Matrix2f& R,
                       const float dt ) {
    // state predict
    Eigen::Vector4f x_pred = motionModel(x_est, u, dt); // 4x1
    Eigen::Matrix4f J_F = jacobianF(x_est, u, dt); //4x4
    Eigen::Matrix4f P_pred = J_F * P_est * J_F.transpose() + Q; // 4x4

    // state update
    Eigen::Vector2f z_pred = observationModel(x_pred);
    Eigen::Vector2f y = z - z_pred;

    Eigen::Matrix<float, 2, 4> J_H = jacobianH();
    Eigen::Matrix2f S = J_H * P_pred * J_H.transpose() + R;
    Eigen::Matrix<float, 4, 2> K  = P_pred *  J_H.transpose() * S.inverse();

    x_est = x_pred + K * y;
    P_est = (Eigen::Matrix4f::Identity()  - K * J_H) * P_pred;
}



// Ellipse generateEllipse(const Eigen::Vector4f& x_est, const Eigen::Matrix4f& P_est) {
//     Ellipse ep;

//     Eigen::Matrix2f p_xy = P_est.block(0, 0, 2, 2); // get cov of x,y
//     Eigen::EigenSolver<Eigen::Matrix2f> ces(p_xy);
//     Eigen::Matrix2f evals = ces.pseudoEigenvalueMatrix();
//     Eigen::Matrix2f evecs = ces.pseudoEigenvectors(); // row major
 
//     // biger eval is 0th idx
//     float a = std::sqrt(evals(0,0));
//     float b = std::sqrt(evals(1,1));

//     // use angle of dominant direction w/ x-axis to determine rotation
//     float angle = std::atan2(evecs(0, 1), evecs(0, 0));
//     Eigen::Matrix2f rot; // rotation matrix
//     rot << std::cos(angle), std::sin(angle),
//            -std::sin(angle), std::cos(angle);


//     for (float t = 0.0; t < 2 * M_PI + 0.1; t += 0.1) {
//         Eigen::Vector2f x;
//         x << a * std::cos(t), b * std::sin(t);
//         Eigen::Vector2f fx = rot * x;
//         ep.emplace_back(fx(0) + x_est(0), fx(1) + x_est(1));
//     }


//     return ep;
// }



