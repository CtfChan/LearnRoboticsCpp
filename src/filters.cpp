#include "filters.hpp"

#include <iostream>

// shared
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


// EKF






// UKF





// PF
ParticleFilter::ParticleFilter(Eigen::Vector4f& init_state, 
    Eigen::Matrix4f& init_cov, Eigen::Matrix2f& R, float Q, float dt, int num_particles) :
    gen_gauss_(rd_gauss_()), gauss_dist_(std::normal_distribution<float>(0.0, 1.0)),
    gen_unif_(rd_unif_()), unif_dist_(std::uniform_real_distribution<float>(1.0, 2.0)),
    R_(R), Q_(Q), dt_(dt),
    num_particles_(num_particles), resample_particles_(num_particles/2.0f) {
    // initliaze particles and weights

    particles_.setZero(4, num_particles);     
    for (int i = 0; i < num_particles_; ++i) particles_.col(i) = init_state;

    weights_.setOnes(num_particles);
    weights_ = weights_ * 1.0f/ num_particles;
}

void ParticleFilter::step(std::vector<LandmarkObserveration>& z, Eigen::Vector2f& u) {
    // predict
    // generate new predicted PDF by passing particle through motion model
    for (int i = 0; i < num_particles_; ++i) {
        Eigen::Vector4f x = particles_.col(i);
        float w = weights_(i);

        Eigen::Vector2f ud;
        ud(0) = u(0) + gauss_dist_(gen_gauss_) * R_(0,0);
        ud(1) = u(1) + gauss_dist_(gen_gauss_) * R_(1,1);

        x = motionModel(x, ud, dt_);

        for (auto [od, ox, oy] : z) {
            float dx = x(0) - ox;
            float dy = x(1) - oy;
            float pre_z = std::hypot(dx, dy);
            w = w * gaussianLikelihood(pre_z, od, std::sqrt(Q_));
        }
        particles_.col(i) = x;
        weights_(i) = w;
    }

    // calculate predict mean and cov from particles
    weights_ = weights_ / weights_.sum();
    state_ = particles_ * weights_;
    cov_ = Eigen::Matrix4f::Zero();
    for (int i = 0; i < num_particles_; ++i) {
        Eigen::Vector4f dx = particles_.col(i) - state_;
        cov_ += weights_(i) * dx * dx.transpose();
    }


    // correction
    // check if necessary, we only resample when Neff < resample_particles
    float Neff = 1.0f / (weights_.transpose() * weights_);
    if (Neff >= resample_particles_) 
        return;
    
    // low variance sampling
    Eigen::VectorXf cum_weights;
    cum_weights.resize(num_particles_);
    cum_weights(0) = weights_(0);
    for (int i = 1; i < num_particles_; ++i)
        cum_weights(i) = cum_weights(i-1) + weights_(i);

    Eigen::VectorXf resampled_id;
    resampled_id.resize(num_particles_);
    for (int i = 0; i < num_particles_; ++i)
        resampled_id(i) = (1.0f / num_particles_) * i + unif_dist_(gen_unif_) / num_particles_;

    Eigen::MatrixXf output;
    output.resize(4, num_particles_);
    int ind = 0;
    for (int i = 0; i < num_particles_; ++i) {
        while (resampled_id(i) > cum_weights(ind) && ind < num_particles_-1 )
            ind += 1;
        output.col(i) = particles_.col(ind);
    }

    particles_ = output;
    weights_.setOnes(num_particles_);
    weights_ = weights_ * (1.f / num_particles_);
}

float ParticleFilter::gaussianLikelihood(float x, float mean, float sigma) {
    float ans = 1.0f / std::sqrt(2.f * M_PI * std::pow(sigma, 2)) * 
                std::exp( -0.5f * std::pow(x - mean, 2) / std::pow(sigma, 2)  );
    return ans;
}


const Eigen::Vector4f ParticleFilter::getState() const {
    return state_;
}

const Eigen::Matrix4f ParticleFilter::getCov() const {
    return cov_;
}






