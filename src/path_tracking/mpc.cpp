#include "path_tracking/mpc.hpp"
#include "common.hpp"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Eigen>


ModelPredictiveController::ModelPredictiveController(int T, int n_ind_search, float dt, float goal_dist) : 
        T_(T), n_ind_search_(n_ind_search), dt_(dt), goal_dist_(goal_dist) {
    x_start_ = 0;
    y_start_ = x_start_ + T;
    yaw_start_ = y_start_ + T;
    v_start_ = yaw_start_ + T;
    delta_start_ = v_start_ + T;
    a_start_ = delta_start_ + T - 1;
    n_vars_ =  T * 4 + (T - 1) * 2;
    n_constraints_ = T * 4;

    xref_.resize(nx_, T); // nx_ = 4
}


 void ModelPredictiveController::setPath(std::vector<float>& cx, std::vector<float>& cy, 
                std::vector<float>& cyaw, std::vector<float>& ck, std::vector<float>& speed_profile)  {
    cx_ = cx;
    cy_ = cy;
    cyaw_ = cyaw;
    ck_ = ck;
    speed_profile_ = speed_profile;

    smoothYaw(cyaw_);

    target_ind_ = 0;
}


std::pair<float, float> ModelPredictiveController::control(BicycleModelRobot& state) {

    calculateReferenceTrajectory(state); //update target_ind_ and xref_

    std::vector<float> output = mpcSolve(state);
    float acc = output[a_start_];
    float steer = output[delta_start_];

    return {acc, steer};
}

bool ModelPredictiveController::atGoal(BicycleModelRobot& state) {
    float dx = state.x - cx_.back();
    float dy = state.y - cy_.back();
    if (std::hypot(dx, dy) <= goal_dist_) {
        return true;
    }

    return false;
}

std::pair<float, float>  ModelPredictiveController::getReferencePosition() {
    float x = cx_[target_ind_];
    float y = cy_[target_ind_];
    return {x, y};
}



void ModelPredictiveController::smoothYaw(std::vector<float> &cyaw) {
    for (unsigned int i = 0; i < cyaw.size() - 1; i++) {
        float dyaw = cyaw[i + 1] - cyaw[i];
        while (dyaw > M_PI / 2.0) {
            cyaw[i + 1] -= M_PI * 2.0;
            dyaw = cyaw[i + 1] - cyaw[i];
        }
        while (dyaw < -M_PI / 2.0) {
            cyaw[i + 1] += M_PI * 2.0;
            dyaw = cyaw[i + 1] - cyaw[i];
        }
    }
}



std::vector<float> calculateSpeedProfile(
    std::vector<float> rx, std::vector<float> ry, std::vector<float> ryaw, float target_speed) {
       // recrod num points
    size_t n = rx.size();
    std::vector<float> speed_profile(n, target_speed);
    bool pos_speed = true;
    for (size_t i = 0; i < n-1; ++i) {
        float dx = rx[i+1] - rx[i];
        float dy = ry[i+1] - ry[i];
        float move_direction = std::atan2(dy, dx);
        // assign negative target speed for large angle 
        if (dx > std::numeric_limits<float>::epsilon() && dy > std::numeric_limits<float>::epsilon()) {
            float dangle = std::abs(normalizeAngle(move_direction - ryaw[i]));
            pos_speed = (dangle >= M_PI / 4.0f) ? false : true;
        }
        speed_profile[i] = pos_speed ? target_speed : -target_speed;
    }
    speed_profile.back() = 0.0; // stop
    return speed_profile;
}



int ModelPredictiveController::calculateNearestIndex(BicycleModelRobot& state, int pind) {
    float mind = std::numeric_limits<float>::max();
    float ind = 0;
    for (unsigned int i = pind; i < pind + n_ind_search_; i++)
    {
        float idx = cx_[i] - state.x;
        float idy = cy_[i] - state.y;
        float d_e = std::hypot(idx, idy);
        if (d_e < mind)
        {
            mind = d_e;
            ind = i;
        }
    }
    return ind;
}


void ModelPredictiveController::calculateReferenceTrajectory(BicycleModelRobot& state) {

    xref_.setZero();

    int ncourse = cx_.size();

    int ind = calculateNearestIndex(state, target_ind_);
    if (target_ind_ >= ind)
        ind = target_ind_;

    xref_(0, 0) = cx_[ind];
    xref_(1, 0) = cy_[ind];
    xref_(2, 0) = cyaw_[ind];
    xref_(3, 0) = speed_profile_[ind];

    float travel = 0.0f;
    for (int i = 0; i < T_; i++)
    {
        travel += std::abs(state.v) * dt_;
        int dind = static_cast<int>(  std::round(travel / dl_)      );
        int new_ind = (ind + dind < ncourse) ? ind + dind : ncourse;

        xref_(0, i) = cx_[new_ind];
        xref_(1, i) = cy_[new_ind];
        xref_(2, i) = cyaw_[new_ind];
        xref_(3, i) = speed_profile_[new_ind];
    }

    target_ind_ = ind;


}



std::vector<float> ModelPredictiveController::mpcSolve(BicycleModelRobot& state) {
    typedef CPPAD_TESTVECTOR(double) Dvector;
    double x = state.x;
    double y = state.y;
    double yaw = state.yaw;
    double v = state.v;

    Dvector vars(n_vars_);
    for (int i = 0; i < n_vars_; i++) {
        vars[i] = 0.0;
    }

    vars[x_start_] = x;
    vars[y_start_] = y;
    vars[yaw_start_] = yaw;
    vars[v_start_] = v;

    // Lower and upper limits for x
    Dvector vars_lowerbound(n_vars_);
    Dvector vars_upperbound(n_vars_);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    // NOTE there mush be both lower and upper bounds for all vars!!!!!
    for (auto i = 0; i < n_vars_; i++)
    {
        vars_lowerbound[i] = -10000000.0;
        vars_upperbound[i] = 10000000.0;
    }

    for (auto i = delta_start_; i < delta_start_ + T_ - 1; i++)
    {
        vars_lowerbound[i] = -  static_cast<double>(state.max_steer);
        vars_upperbound[i] = static_cast<double>(state.max_steer);
    }

    for (auto i = a_start_; i < a_start_ + T_ - 1; i++)
    {
        vars_lowerbound[i] = -static_cast<double>(state.max_accel);
        vars_upperbound[i] = static_cast<double>(state.max_accel);
    }

    for (auto i = v_start_; i < v_start_ + T_; i++)
    {
        vars_lowerbound[i] = static_cast<double>(state.min_speed);
        vars_upperbound[i] = static_cast<double>(state.max_speed);
    }

    Dvector constraints_lowerbound(n_constraints_);
    Dvector constraints_upperbound(n_constraints_);
    for (auto i = 0; i < n_constraints_; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[x_start_] = x;
    constraints_lowerbound[y_start_] = y;
    constraints_lowerbound[yaw_start_] = yaw;
    constraints_lowerbound[v_start_] = v;

    constraints_upperbound[x_start_] = x;
    constraints_upperbound[y_start_] = y;
    constraints_upperbound[yaw_start_] = yaw;
    constraints_upperbound[v_start_] = v;

    // FG_EVAL fg_eval;
    FUNC_EVAL func_eval {T_, dt_, state.L, x_start_, y_start_, yaw_start_, v_start_, delta_start_, a_start_, xref_ };

    // options
    std::string options;
    options += "Integer print_level  0\n";
    // options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    options += "Integer max_iter      50\n";
    // options += "Numeric tol          1e-6\n";
    options += "Numeric max_cpu_time          0.05\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    CppAD::ipopt::solve<Dvector, FUNC_EVAL>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, func_eval, solution);


    bool ok = (solution.status == CppAD::ipopt::solve_result<Dvector>::success);

    std::cout << "Optimization success?: " << ok << std::endl;


    std::vector<float> result;
    for (auto i = 0; i < n_vars_; i++)
    {
        result.push_back((float)solution.x[i]);
    }
    return result;
}



void FUNC_EVAL::operator()(ADvector &fg, const ADvector &vars) {
    fg[0] = 0;

    for (int i = 0; i < T_ - 1; i++)
    {
        fg[0] += 0.01 * CppAD::pow(vars[a_start_ + i], 2);
        fg[0] += 0.01 * CppAD::pow(vars[delta_start_ + i], 2);
    }

    for (int i = 0; i < T_ - 2; i++)
    {
        fg[0] += 0.01 * CppAD::pow(vars[a_start_ + i + 1] - vars[a_start_ + i], 2);
        fg[0] += 1 * CppAD::pow(vars[delta_start_ + i + 1] - vars[delta_start_ + i], 2);
    }

    // fix the initial state as a constraint
    fg[1 + x_start_] = vars[x_start_];
    fg[1 + y_start_] = vars[y_start_];
    fg[1 + yaw_start_] = vars[yaw_start_];
    fg[1 + v_start_] = vars[v_start_];

    // The rest of the constraints
    for (int i = 0; i < T_ - 1; i++)
    {
        // The state at time t+1 .
        AD<double> x1 = vars[x_start_ + i + 1];
        AD<double> y1 = vars[y_start_ + i + 1];
        AD<double> yaw1 = vars[yaw_start_ + i + 1];
        AD<double> v1 = vars[v_start_ + i + 1];

        // The state at time t.
        AD<double> x0 = vars[x_start_ + i];
        AD<double> y0 = vars[y_start_ + i];
        AD<double> yaw0 = vars[yaw_start_ + i];
        AD<double> v0 = vars[v_start_ + i];

        // Only consider the actuation at time t.
        AD<double> delta0 = vars[delta_start_ + i];
        AD<double> a0 = vars[a_start_ + i];

        // constraint with the dynamic model
        fg[2 + x_start_ + i] = x1 - (x0 + v0 * CppAD::cos(yaw0) * dt_);
        fg[2 + y_start_ + i] = y1 - (y0 + v0 * CppAD::sin(yaw0) * dt_);
        fg[2 + yaw_start_ + i] = yaw1 - (yaw0 + v0 * CppAD::tan(delta0) / L_ * dt_);
        fg[2 + v_start_ + i] = v1 - (v0 + a0 * dt_);
        // cost with the ref traj
        fg[0] += CppAD::pow(xref_(0, i + 1) - (x0 + v0 * CppAD::cos(yaw0) * dt_), 2);
        fg[0] += CppAD::pow(xref_(1, i + 1) - (y0 + v0 * CppAD::sin(yaw0) * dt_), 2);
        fg[0] += 0.5 * CppAD::pow(xref_(2, i + 1) - (yaw0 + v0 * CppAD::tan(delta0) / L_ * dt_), 2);
        fg[0] += 0.5 * CppAD::pow(xref_(3, i + 1) - (v0 + a0 * dt_), 2);
    }
}