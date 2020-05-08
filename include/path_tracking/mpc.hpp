#pragma once

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Eigen>

using CppAD::AD;

typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
typedef CPPAD_TESTVECTOR(double) Dvector;



std::vector<float> calculateSpeedProfile(std::vector<float>& rx, 
            std::vector<float>& ry, std::vector<float>& ryaw, float target_speed);


void smoothYaw(std::vector<float>& yaw);

size_t calculateNearestIndex(BicycleModelRobot& state, std::vector<float>& x, 
                        std::vector<float>& y, std::vector<float>& yaw, size_t pind, size_t n_ind);


class FG_EVAL{
public:

    // FG_EVAL(Eigen::MatrixXf traj_ref) : traj_ref_(traj_ref) {}
    FG_EVAL(Eigen::MatrixXf traj_ref, float dt, float L);

    void operator()(ADvector& fg, const ADvector& vars);

private:
    Eigen::MatrixXf traj_ref_; 
    float dt_;
    float L_;
};


// class ModelPredictiveController {
// public:


// private:

//     // trajectory to follow
//     std::vector<float> cx_, cy_, cyaw_;


// };