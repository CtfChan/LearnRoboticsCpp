# pragma once

#include <vector>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Eigen>

#include "common.hpp"

using CppAD::AD;


class ModelPredictiveController {
public:
    ModelPredictiveController(int T, int n_ind_search, float dt) : 
            T_(T), dt_(dt) {
        x_start_ = 0;
        y_start_ = x_start_ + T;
        yaw_start_ = y_start_ + T;
        v_start_ = yaw_start_ + T;
        delta_start_ = v_start_ + T;
        a_start_ = delta_start_ + T - 1;
        n_vars_ =  T * 4 + (T - 1) * 2;
        n_constraints_ = T * 4;
    }

    void setPath(std::vector<float>& cx, std::vector<float>& cy, std::vector<float>& cyaw);

    std::pair<float, float> control(BicycleModelRobot& state);

    bool atGoal();

private:

    void smoothYaw(std::vector<float> &cyaw);

    std::vector<float> calculateSpeedProfile(std::vector<float> rx, std::vector<float> ry, 
                                            std::vector<float> ryaw, float target_speed);


    int calculateNearestIndex(BicycleModelRobot& state, int pind);


    std::pair<Eigen::MatrixXf, int> calculateReferenceTrajectory();


    float dt_;

    Eigen::MatrixXf xref_;
    int target_ind_;

    int nx_ = 4; // fixed number of states
    int T_; // time horizon can be varies

    int x_start_;
    int y_start_;
    int yaw_start_;
    int v_start_;
    int delta_start_;
    int a_start_;

    size_t n_vars_, n_constraints_;

    bool at_goal_ = true;

    // define nested class for optimization
    class FUNC_EVAL {
        public:
            typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
            void operator()(ADvector &fg, const ADvector &vars);
    };
};


