#pragma once

#include <Eigen/Eigen>
#include <random>

using Path = std::vector<std::pair<float, float>>;
using Ellipse =  std::vector<std::pair<float, float>>;


// shared
Eigen::Vector4f motionModel(const Eigen::Vector4f& x, const Eigen::Vector2f& u, const float dt);

Eigen::Vector2f observationModel(const Eigen::Vector4f& x);

Ellipse generateEllipse(const Eigen::Vector4f& x_est, const Eigen::Matrix4f& P_est);



// EKF




// UKF



// PF
struct LandmarkObserveration {
    float distance;
    float x;
    float y;
};

class ParticleFilter {
public:
    ParticleFilter(Eigen::Vector4f& init_state, Eigen::Matrix4f& init_cov,
                   Eigen::Matrix2f& R, float Q, float dt, int num_particles);

    void step(std::vector<LandmarkObserveration>& z, Eigen::Vector2f& u);

    const Eigen::Vector4f getState() const;

    const Eigen::Matrix4f getCov() const;

private:
    float gaussianLikelihood(float x, float mean, float sigma);

    Eigen::MatrixXf particles_;
    Eigen::VectorXf weights_;

    Eigen::Vector4f state_;
    Eigen::Matrix4f cov_;

    Eigen::Matrix2f& R_;
    float Q_;

    float dt_;

    std::random_device rd_gauss_;
    std::mt19937 gen_gauss_;
    std::normal_distribution<float> gauss_dist_;

    std::random_device rd_unif_;
    std::mt19937 gen_unif_;
    std::uniform_real_distribution<float> unif_dist_;

    int num_particles_;
    int resample_particles_;

};


