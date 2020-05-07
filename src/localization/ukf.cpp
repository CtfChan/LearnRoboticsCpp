#include "localization/ukf.hpp"


// UKF

std::tuple<std::vector<float>, std::vector<float>, float> 
    setupWeights(float nx, float alpha, float kappa, float beta) {
    // nx = L
    float lamb = std::pow(alpha, 2) * (nx + kappa) - nx;
    float gamma = std::sqrt(nx + lamb);

    std::vector<float> wm(2*nx+1, 0.0f);
    std::vector<float> wc(2*nx+1, 0.0f);
    
    wm[0] = lamb / (lamb + nx);
    wc[0] = ( lamb / (lamb + nx) ) + (1 - std::pow(alpha, 2) + beta);

    for (size_t i = 1; i < 2*nx+1; ++i) {
        wm[i] = 1.0f / (2 * (nx + lamb));
        wc[i] = 1.0f / (2 * (nx + lamb));
    }

    return {wm, wc, gamma};
}

void unscentedKalmanFilter(Eigen::Vector4f& x_est, Eigen::Matrix4f& P_est,
        const Eigen::Vector2f& u, const Eigen::Vector2f& z, 
        const Eigen::Matrix4f& Q, const Eigen::Matrix2f& R,
        const float dt, const float gamma,
        const std::vector<float>& wm, 
        const std::vector<float>& wc) {
    std::vector<Eigen::Vector4f> sigma_points;

    // STATE PREDICT
    // generate and propogate sigma points through motion model
    sigma_points = generateSigmaPoints(x_est, P_est, gamma);
    for (auto& pt : sigma_points)
        pt = motionModel(pt, u, dt);

    // recombine sigma points to get predict mean and cov
    Eigen::Vector4f x_pred = Eigen::Vector4f::Zero(4);
    for (size_t i = 0; i < sigma_points.size(); ++i)
        x_pred += wm[i] * sigma_points[i];
    
    Eigen::Matrix4f P_pred = Q;
    for (size_t i = 0; i < sigma_points.size(); ++i) {
        Eigen::Vector4f diff = (sigma_points[i] - x_pred);
        P_pred += wc[i] * diff * diff.transpose();
    }

    // STATE CORRECT
    Eigen::Vector2f z_pred = observationModel(x_pred);
    Eigen::Vector2f y = z - z_pred;

    // generate sigma points, transform to observation domain
    sigma_points = generateSigmaPoints(x_pred, P_pred, gamma);
    std::vector<Eigen::Vector2f> z_sigma_points;
    for (auto& pt : sigma_points)
        z_sigma_points.emplace_back( observationModel(pt) );

    Eigen::Vector2f z_mean = Eigen::Vector2f::Zero(2);
    for (size_t i = 0; i < sigma_points.size(); ++i)
        z_mean += wm[i] * z_sigma_points[i];

    Eigen::Matrix2f cov_zz = R; // st
    for (size_t i = 0; i < z_sigma_points.size(); ++i) {
        Eigen::Vector2f diff = (z_sigma_points[i] - z_mean);
        cov_zz += wc[i] * diff * diff.transpose();
    }

    // Eigen::Matrix<float, 4, 2> cov_xz;
    Eigen::Matrix<float, 4, 2> cov_xz = Eigen::MatrixXf::Zero(4,2);
    for (size_t i = 0; i < z_sigma_points.size(); ++i) {
        Eigen::Vector2f diffz = (z_sigma_points[i] - z_mean);
        Eigen::Vector4f diffx = (sigma_points[i] - x_pred);
        cov_xz += wc[i] * diffx * diffz.transpose();
    }         

    auto K = cov_xz * cov_zz.inverse(); // (4x2) * (2,2) => (4,2)
    x_est = x_pred + K * y; // (4,1) + (4,2) * (2,1)
    P_est = P_pred - K * cov_zz * K.transpose(); // (4,4) - (4,2)*(2,2)*(2,4)   
}


 std::vector<Eigen::Vector4f> generateSigmaPoints(
     const Eigen::Vector4f& x_est, const Eigen::Matrix4f& P_est, const float gamma) {
    //
    const size_t L = 4;
    std::vector<Eigen::Vector4f> sigma_points;
    sigma_points.reserve(2*L + 1);

    sigma_points.push_back(x_est);

    Eigen::Matrix4f P_sqrt ( P_est.llt().matrixL() );
    for (size_t i = 0; i < L; ++i) {
        Eigen::Vector4f pt = x_est + gamma * P_sqrt.col(i);
        sigma_points.push_back(pt);
    }    

    for (size_t i = 0; i < L; ++i) {
        Eigen::Vector4f pt = x_est - gamma * P_sqrt.col(i);
        sigma_points.push_back(pt);
    }    

    return sigma_points;

}
