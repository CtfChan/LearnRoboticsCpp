# pragma once

#include <vector>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Eigen>

#include "common.hpp"

using CppAD::AD;


std::vector<float> calculateSpeedProfile(std::vector<float> rx, std::vector<float> ry, 
                                        std::vector<float> ryaw, float target_speed);


class ModelPredictiveController {
public:
    ModelPredictiveController(int T, int n_ind_search, float dt, float goal_dist);
    
    void setPath(std::vector<float>& cx, std::vector<float>& cy, std::vector<float>& cyaw,
                std::vector<float>& ck, std::vector<float>& speed_profile);

    std::pair<float, float> control(BicycleModelRobot& state);

    std::pair<float, float>  getReferencePosition();

    bool atGoal(BicycleModelRobot& state) ;

private:

    void smoothYaw(std::vector<float> &cyaw) ;

    int calculateNearestIndex(BicycleModelRobot& state, int pind);

    void calculateReferenceTrajectory(BicycleModelRobot& state);

    std::vector<float> mpcSolve(BicycleModelRobot& state);

    std::vector<float> cx_, cy_, cyaw_, ck_;

    float dt_;

    float dl_ = 1.f;
    Eigen::MatrixXf xref_;
    int target_ind_;
    int n_ind_search_;

    int nx_ = 4; // fixed number of states
    int T_; // time horizon can be varies

    int x_start_;
    int y_start_;
    int yaw_start_;
    int v_start_;
    int delta_start_;
    int a_start_;

    size_t n_vars_, n_constraints_;

    float goal_dist_;

    std::vector<float> speed_profile_;

};


struct FUNC_EVAL {
public:
//     FUNC_EVAL();

// float T_;
//     float dt_;
//     float L_;
//     float x_start_, y_start_, yaw_start_, v_start_, delta_start_, a_start_;
//     Eigen::MatrixXf xref_;


    int T_;
    double dt_;
    double L_;
    int x_start_, y_start_, yaw_start_, v_start_, delta_start_, a_start_;
    Eigen::MatrixXf xref_;

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector &fg, const ADvector &vars);
};
