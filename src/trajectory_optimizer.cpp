#include "trajectory_optimizer.hpp"

#include <iostream>

std::array<float, 3> quadraticCoefficients(std::array<float, 3>& x, std::array<float, 3>& y) {
    Eigen::Matrix3f A;
    Eigen::Vector3f b;
    A << std::pow(x[0], 2), x[0], 1,
        std::pow(x[1], 2), x[1], 1,
        std::pow(x[2], 2), x[2], 1;
    b << y[0], y[1], y[2];

    Eigen::Vector3f coeffs = A.inverse() * b;
    
    return {coeffs[0] , coeffs[1], coeffs[2]};
}

float quadraticInterpolation(std::array<float, 3>& coeff, float x) {
    return coeff[0] * std::pow(x, 2) + coeff[1] * x + coeff[2];
}


TrajectoryOptimizer::TrajectoryOptimizer(BicycleModelRobot& state, size_t max_iter, float cost_th, 
    TrajectoryParam& delta_params, float ds, float min_alpha, float max_alpha, float d_alpha) :
    init_state_(state), max_iter_(max_iter), cost_th_(cost_th), delta_params_(delta_params), 
    ds_(ds), min_alpha_(min_alpha), max_alpha_(max_alpha), d_alpha_(d_alpha) {

}



  
Pose2DTrajectory TrajectoryOptimizer::optimizeTrajectory(Pose2D& target_pose, TrajectoryParam params, float k0, Gnuplot& gp) {
    Pose2DTrajectory traj;

    for (size_t i = 0; i < max_iter_; ++i) {
        traj = generateTrajectory(params, k0);
        Pose2D final_pose {traj.x.back(), traj.y.back(), traj.theta.back()};
        Eigen::Vector3f d = calculatePoseErrorVector(target_pose, final_pose);
        float cost = d.norm();
        std::cout << "final_pose: " << final_pose.x << " " << final_pose.y << " " << final_pose.theta << std::endl;

        std::cout << "current cost " << cost << std::endl;

        gp << "plot '-' with line \n";
        gp.send1d(boost::make_tuple(traj.x, traj.y));
        sleep(1);

        if (cost < cost_th_) {
            std::cout << "Trajectory found!" << std::endl;
            break;
        }

        Eigen::Matrix3f J = calculateJacobian(target_pose, params, k0);
        Eigen::Vector3f dp = - J.inverse() * d;
        float alpha = selectLearningParam(target_pose, params, k0, dp);
        params.s += alpha * dp(0);
        params.km += alpha * dp(1);
        params.kf += alpha * dp(2);

        // std::cout << "alpha: " << alpha << std::endl;
        // std::cout << "params: " << params.s  << " " << params.km << " " << params.kf << std::endl;

        std::cout << "dp: " << dp << std::endl;
        std::cout << "jacob: " << J  << std::endl;

    }



    return traj;
}

float TrajectoryOptimizer::selectLearningParam(Pose2D& target_pose, TrajectoryParam params, float k0, Eigen::Vector3f& dp) {
    float min_cost = std::numeric_limits<float>::max();
    float min_a = min_alpha_;
    for (float a = min_alpha_; a < max_alpha_; a += d_alpha_) {
        TrajectoryParam new_params = params;
        new_params.s += a * dp(0);
        new_params.km += a * dp(1);
        new_params.kf += a * dp(2);
        Pose2D last_pose = generateFinalState(new_params, k0);
        Eigen::Vector3f dstate = calculatePoseErrorVector(target_pose, last_pose);
        
        float cost = dstate.norm();
        if (cost <= min_cost && std::abs(cost) > std::numeric_limits<float>::epsilon()) {
            min_a = a;
            min_cost = cost;
        }
    
    }

    return min_a;

}


Eigen::MatrixXf TrajectoryOptimizer::calculateJacobian(Pose2D& target_pose, TrajectoryParam& params, float k0) {
    // perturb each param, use centered diff to approx jacobian
    TrajectoryParam params_pert; 
    Eigen::Vector3f dp, dn;
    Pose2D pose_pos, pose_neg; // pose from positive and negative perturbation

    // perturb s, first col of jacobian
    params_pert = {params.s + delta_params_.s, params.km, params.kf};
    pose_pos = generateFinalState(params_pert, k0);
    dp = calculatePoseErrorVector(target_pose, pose_pos);
    
    params_pert = {params.s - delta_params_.s, params.km, params.kf};
    pose_neg = generateFinalState(params_pert, k0);
    dn = calculatePoseErrorVector(target_pose, pose_neg);

    Eigen::Vector3f d1 = (dp - dn) / (2.0f * delta_params_.s); 
    
    // perturb km, second col
    params_pert = {params.s, params.km + delta_params_.km, params.kf};
    pose_pos = generateFinalState(params_pert, k0);
    dp = calculatePoseErrorVector(target_pose, pose_pos);
    
    params_pert = {params.s, params.km - delta_params_.km, params.kf};
    pose_neg = generateFinalState(params_pert, k0);
    dn = calculatePoseErrorVector(target_pose, pose_neg);

    Eigen::Vector3f d2 = (dp - dn) / (2.0f * delta_params_.km); 
    
    // perturb kf, third col of jacobian
    params_pert = {params.s, params.km, params.kf + delta_params_.kf};
    pose_pos = generateFinalState(params_pert, k0);
    dp = calculatePoseErrorVector(target_pose, pose_pos);
    
    params_pert = {params.s, params.km, params.kf - delta_params_.kf};
    pose_neg = generateFinalState(params_pert, k0);
    dn = calculatePoseErrorVector(target_pose, pose_neg);

    Eigen::Vector3f d3 = (dp - dn) / (2.0f * delta_params_.kf); 
    
    // return matrix
    Eigen::Matrix3f J;
    J << d1, d2, d3;
    return J;
}


Pose2DTrajectory TrajectoryOptimizer::generateTrajectory(TrajectoryParam& params, float k0) {
    Pose2DTrajectory traj;

    float n = params.s / ds_; // number of discrete points along trajectory to acquire
    float time_horizon = params.s / init_state_.v;
    float dt = time_horizon / n; 

     // make copy of init state and update this copy
    BicycleModelRobot state = init_state_;
    traj.x.push_back(state.x);
    traj.y.push_back(state.x);
    traj.theta.push_back(state.yaw);

    std::array<float, 3> xk = {0.0f, time_horizon/2.0f, time_horizon};
    std::array<float, 3> yk = {k0, params.km, params.kf};
    std::array<float, 3> coeff = quadraticCoefficients(xk, yk);

    std::cout << "x: " << xk[0] << " " << xk[1] << " " << xk[2] << std::endl;
    std::cout << "y: " << yk[0] << " " << yk[1] << " " << yk[2] << std::endl;
    std::cout << "coeff: " << coeff[0] <<  " "  << coeff[1] << " " << coeff[2] << std::endl;
    std::cout << "n : " << n <<  std::endl;
    std::cout << "time_horizon: " << time_horizon << std::endl;


    for (float t = 0.f; t < time_horizon; t += dt) {
        float steer = quadraticInterpolation(coeff, t);
        state.update(0.0f, steer, dt); // no acc.
        traj.x.push_back(state.x);
        traj.y.push_back(state.y);
        traj.theta.push_back(state.yaw);
    }

    return traj;
}

Pose2D TrajectoryOptimizer::generateFinalState(TrajectoryParam& params, float k0) {
    float n = params.s / ds_; // number of discrete points along trajectory to acquire
    float time_horizon = params.s / init_state_.v;
    float dt = time_horizon / n; 

     // make copy of init state and update this copy
    BicycleModelRobot state = init_state_;
  
    std::array<float, 3> xk = {0.0f, time_horizon/2.0f, time_horizon};
    std::array<float, 3> yk = {k0, params.km, params.kf};
    std::array<float, 3> coeff = quadraticCoefficients(xk, yk);

    for (float t = 0.f; t < time_horizon; t += dt) {
        float steer = quadraticInterpolation(coeff, t);
        state.update(0.0f, steer, dt); // no acc.
    }

    return {state.x, state.y, state.yaw};
}



Eigen::Vector3f TrajectoryOptimizer::calculatePoseErrorVector(const Pose2D& p1, const Pose2D p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dtheta = normalizeAngle(p1.theta - p2.theta);
    
    Eigen::Vector3f d;
    d << dx, dy, dtheta;

    return d;
}
