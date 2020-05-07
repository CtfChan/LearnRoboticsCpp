#include "common.hpp"

#include "localization/ekf.hpp"
#include "localization/filters.hpp"

#include "gnuplot-iostream.h"

#include <random>


int main(){

    // simulation parameters
    float sim_time = 50.0f; // s
    float dt = 0.1;

    // control input (v, w) will be constant 
    Eigen::Vector2f u = Eigen::Vector2f(1.0f, 0.1f);

    // noisy control for dr
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

    // initialize plot
    Gnuplot gp;
    Path gt_path;
    Path dr_path;
    Path obs_path;
    Path est_path;

    gp << "set size ratio 1.0\n";
    gp << "set term gif animate\n";
    gp << "set output '../animations/ekf.gif'\n";


    float time = 0.0f;
    while(time <= sim_time){
        time += dt;

        // propagate true motion
        x_GT = motionModel(x_GT, u, dt);
        gt_path.emplace_back(x_GT(0), x_GT(1));

        // propogate noisy dead reckoning
        ud(0) = u(0) + gaussian(generator) * control_noise_v;
        ud(1) = u(1) + gaussian(generator) * control_noise_w;

        x_DR = motionModel(x_DR, ud, dt);
        dr_path.emplace_back(x_DR(0), x_DR(1));

        // generate observation using ground truth + noise
        z(0) = x_GT(0) + gaussian(generator) * obs_noise_x;
        z(1) = x_GT(1) + gaussian(generator) * obs_noise_y;
        obs_path.emplace_back(z(0), z(1));


        // do ekf
        extendedKalmanFilter(x_est, P_est, ud, z, Q, R, dt);
        est_path.emplace_back(x_est(0), x_est(1));

        // generate error ellipse
        Ellipse error_ellipse = generateEllipse(x_est, P_est);

        gp << "plot '-' title 'ground truth' with lines,"
                    "'-' title 'odometry' with lines,"
                    "'-' title 'observation',"
                    "'-' title 'covariance' with lines,"
                    "'-' title 'filter' with lines\n";
        gp.send1d(gt_path);
        gp.send1d(dr_path);
        gp.send1d(obs_path);
        gp.send1d(error_ellipse);
        gp.send1d(est_path);
        // sleep(0.2);        

    }

    gp << "set output\n";


}
