#include "filters.hpp"

#include "common.hpp"

#include "gnuplot-iostream.h"

int main() {
    // simulation parameters
    float sim_time = 50.0f; // s
    float dt = 0.1f;

    // PF specific parms
    float max_range = 20.f; // maximum range of id
    int num_particles = 100; // number of particles
    // std::vector<std::pair<float, float> > rf_ids = {
    //     {10.f, 0.f},
    //     {10.f, 10.f},
    //     {0.f, 15.f},
    //     {-5.f, 20.f}
    // };
    Eigen::Matrix<float, 4, 2> rf_ids;
    rf_ids << 10.0f, 0.0f,
              10.0f, 10.0f,
               0.0f, 15.0f,
              -5.0f, 20.0f;


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
    float Q = 0.1;

    // observation model covariance
    Eigen::Matrix2f  R = Eigen::Matrix2f::Identity();
    R(0,0) = 1.0;
    R(1,1) = std::pow(deg2rad(40.f), 2);

    // Motion model simulation error
    float Qsim = 0.04f;

    // Observation model simulation error
    Eigen::Matrix2f Rsim = Eigen::Matrix2f::Identity();
    Rsim(0,0) = 1.0f * 1.0f;
    Rsim(1,1) = std::pow(deg2rad(30.f),2);

    // random number generator
    std::random_device random_device{};
    std::mt19937 generator{random_device()};
    std::normal_distribution<> gaussian(0, 1);

    // initialize filter
    ParticleFilter pf(x_est, P_est, num_particles);

    // initialize plot
    Gnuplot gp;
    Path gt_path;
    Path dr_path;
    Path obs_path;
    Path est_path;

    // gp << "set size ratio 1.0\n";
    // gp << "set term gif animate\n";
    // gp << "set output '../animations/particle_filter.gif'\n";


    float time = 0.f;
    while (time < sim_time) {
        time += dt;

        // propagate true motion
        x_GT = motionModel(x_GT, u, dt);
        gt_path.emplace_back(x_GT(0), x_GT(1));

        // propogate noisy dead reckoning
        ud(0) = u(0) + gaussian(generator) * Rsim(0,0);
        ud(1) = u(1) + gaussian(generator) * Rsim(1,1);

        x_DR = motionModel(x_DR, ud, dt);
        dr_path.emplace_back(x_DR(0), x_DR(1));

        // generate observation
        std::vector<LandmarkObserveration> curr_z;
        for (size_t i = 0; i < rf_ids.rows(); ++i) {
            Eigen::Vector2f pos = rf_ids.row(i);
            float d = (x_est.head(2) - pos).norm();
            if (d <= max_range) {
                float dn = d + gaussian(generator) * Qsim;
                LandmarkObserveration obs{dn, pos(0), pos(1)};
                curr_z.push_back(obs);
            }
        }
        
        pf.step(curr_z, ud);


    }




    return 0;
}