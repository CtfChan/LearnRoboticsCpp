#include<iostream>
#include<random>
#include<cmath>

#include<Eigen/Eigen>

#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

// simulation parameters
float sim_time = 50.0f;
float dt = 0.1;

// constants 
size_t L = 4;
float kappa = 3.0f;


// x is state, u is control
Eigen::Vector4f motionModel(const Eigen::Vector4f& x, const Eigen::Vector2f& u) {
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


float deg2rad(float deg) {
    return (deg / 180.f * M_PI);
}

 std::vector<Eigen::Vector4f> generateSigmaPoints(Eigen::Vector4f& x_est, Eigen::Matrix4f& P_est) {


    std::vector<Eigen::Vector4f> sigma_points;
    sigma_points.reserve(2*L + 1);

    sigma_points.push_back(x_est);

    Eigen::MatrixXd chol_L ( P_est.llt().matrixL() );
    for (size_t i = 0; i < L; ++i) {
        Eigen::Vector4f pt = x_est + std::sqrt(L + kappa) * chol_L.col(i);
        sigma_points.push_back(pt);
    }    

    for (size_t i = 0; i < L; ++i) {
        Eigen::Vector4f pt = x_est - std::sqrt(L + kappa) * chol_L.col(i);
        sigma_points.push_back(pt);
    }    

    return sigma_points;

 }

void unscentedKalmanFilter(Eigen::Vector4f& x_est, Eigen::Matrix4f& P_est,
                          const Eigen::Vector2f& u, const Eigen::Vector2f& z, 
                          const Eigen::Matrix4f& Q, const Eigen::Matrix2f& R) {
    std::vector<Eigen::Vector4f> sigma_points;

    // STATE PREDICT
    // generate sigma points from mean and covariance 
    sigma_points = generateSigmaPoints(x_est, P_est);

    // pass each sigma point through motion model
    for (auto& pt : sigma_points) {
        pt = motioModel(pt, u);
    }

    // recombine sigma points
    Eigen::Vector4f x_pred = Eigen::Vector4f::Zero(4);
    for (size_t i = 0; i < sigma_points.size(); ++i) {
        if (i == 0) {
            x_pred += (kappa / (L + kappa)) * sigma_points[i];
        } else {
            x_pred += (1.0 / 2.0*(L + kappa)) * sigma_points[i];
        }
    }

    Eigen::Matrix4f P_pred = Q;
    for (size_t i = 0; i < sigma_points.size(); ++i) {
        Eigen::Vector4f diff = (sigma_points[i] - x_pred);
        if (i == 0) {
            P_pred += (kappa / (L + kappa)) * diff * diff.transpose();
        } else {
            P_pred += (1.0 / 2.0*(L + kappa)) *diff * diff.transpose();
        }
    }

    // STATE CORRECT
    // make observation
    Eigen::Vector2f z_pred = observationModel(x_pred);

    // generate sigma points from predicted state and covariance
    sigma_points = generateSigmaPoints(x_pred, P_pred);

    // pass sigma point through observation model
    for (auto& pt : sigma_points) {
        pt = observationModel(pt);
    }

    // recombine sigma points to get required moments
    // mu_y
    // sigma_yy
    // sigma_xy

    // K_k = sigma_xy * sigma_yy.inverse()
    // P_est = P_pred - K * sima_xy.transpose()
    // x_est = x_pred + K * (y - mu_y)

}





int main(){
    // control input (v, w) will be constant 
    Eigen::Vector2f u = Eigen::Vector2f(1.0f, 0.1f);

    // control input (v, w) will be constant 
    Eigen::Vector2f ud = Eigen::Vector2f(1.0f, 0.1f);

    // dead reckoning state (x, y, v, w)
    Eigen::Vector4f x_DR = Eigen::Vector4f::Zero(4);

    // ground truth state (x, y, v, w)
    Eigen::Vector4f x_GT = Eigen::Vector4f::Zero(4);

    // observation z (x, y) coordinates
    Eigen::Vector2f z;

    // estimation of state, covariance of state
    Eigen::Vector4f x_est = Eigen::Vector4f::Zero(4);
    Eigen::Matrix4f P_est = Eigen::Matrix4f::Identity();

    // motion model covariance
    Eigen::Matrix4f Q = Eigen::Matrix4f::Identity();
    Q(0,0) = 0.1 * 0.1;
    Q(1,1) = 0.1 * 0.1;
    Q(2,2) =std::pow(deg2rad(1.0), 2)  ;
    Q(3,3) = 0.1 * 0.1;

    // observation model covariance
    Eigen::Matrix2f  R = Eigen::Matrix2f::Identity();
    R(0,0)=1.0;
    R(1,1)=1.0;

    float control_noise_v = 1.0;
    float control_noise_w = std::pow(deg2rad(30.0), 2); 

    float obs_noise_x =  0.5 * 0.5;
    float obs_noise_y = 0.5 * 0.5;

    // random number generator
    std::random_device random_device{};
    std::mt19937 generator{random_device()};
    std::normal_distribution<> gaussian(0, 1);


    // visualization
    cv::Mat viz(500, 500, CV_8UC3, cv::Scalar(255,255,255));


    float time = 0.0f;
    while(time <= sim_time){
        time += dt;

        // propagate true motion
        x_GT = motionModel(x_GT, u);

        // propogate noisy dead reckoning
        ud(0) = u(0) + gaussian(generator) * control_noise_v;
        ud(1) = u(1) + gaussian(generator) * control_noise_w;

        x_DR = motionModel(x_DR, ud);

        // generate observation using ground truth + noise
        z(0) = x_GT(0) + gaussian(generator) * obs_noise_x;
        z(1) = x_GT(1) + gaussian(generator) * obs_noise_y;

        // do UKF
       
    
        cv::imshow("ukf", viz);
        cv::waitKey(5);
    }

}

