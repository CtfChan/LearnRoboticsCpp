#pragma once

#include "common.hpp"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Eigen>

using CppAD::AD;

typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
typedef CPPAD_TESTVECTOR(double) Dvector;

std::vector<float> calculateSpeedProfile(std::vector<float>& rx, 
            std::vector<float>& ry, std::vector<float>& ryaw, float target_speed);

void smoothYaw(std::vector<float>& yaw);

int calculateNearestIndex(BicycleModelRobot& state, std::vector<float>& x, 
                        std::vector<float>& y, std::vector<float>& yaw, int pind, int n_ind);

void calculateReferenceTrajectory(BicycleModelRobot& state, std::vector<float>& cx, 
    std::vector<float>& cy, std::vector<float>& cyaw, std::vector<float>& ck,
    std::vector<float>& sp, float dt, float dl, 
    Eigen::MatrixXf& xref, int& target_ind, int n_ind);

std::vector<float> mpcSolve(BicycleModelRobot& state, Eigen::MatrixXf& xref, float dt);




class FG_EVAL{
public:

    FG_EVAL(Eigen::MatrixXf traj_ref, float dt, float L);

    void operator()(ADvector& fg, const ADvector& vars);


    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

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