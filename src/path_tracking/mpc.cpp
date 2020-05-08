#include "path_tracking/mpc.hpp"
#include "common.hpp"

std::vector<float> calculateSpeedProfile(std::vector<float>& rx, 
            std::vector<float>& ry, std::vector<float>& ryaw, float target_speed) {
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
            pos_speed = (dangle >= M_PI  / 4.0f) ? false : true;
        }
        speed_profile[i] = pos_speed ? target_speed : -target_speed;
    }

    speed_profile.back() = 0.0; // stop

    return speed_profile;
}


void smoothYaw(std::vector<float>& yaw) {
    for (size_t i = 0; i < yaw.size()-1; ++i) {
        float dyaw = yaw[i+1] - yaw[i];

        while (dyaw > M_PI/2.f) {
            yaw[i+1]-= 2.f * M_PI;
            dyaw = yaw[i+1] - yaw[i];
        }

        while (dyaw < -M_PI/2.f) {
            yaw[i+1] += M_PI*2.f;
            dyaw = yaw[i+1] - yaw[i];
        }
    }
}

int calculateNearestIndex(BicycleModelRobot& state, std::vector<float>& x, 
                        std::vector<float>& y, std::vector<float>& yaw, int pind, int n_ind){
    //
    float best_d = std::numeric_limits<float>::max();
    int best_ind = pind;
    for (int i = pind; i < pind + n_ind; ++i ) {
        float dx = state.x - x[i];
        float dy = state.y - y[i];
        float d = std::hypot(dx, dy);
        if (d < best_id){
            best_ind = i;
            best_d = d;
        }
    }

    return best_ind;
}



void calculateReferenceTrajectory(BicycleModelRobot& state, std::vector<float>& cx, 
    std::vector<float>& cy, std::vector<float>& cyaw, std::vector<float>& ck,
    std::vector<float>& sp, float dt, float dl, 
    Eigen::MatrixXf& xref, int& target_ind) {
    
    int ncourse = x.size();
    
    xref.setZero();

    int ind = calculateNearestIndex(state, cx, cy, cyaw, target_ind);
    if (target_ind >= ind) ind = target_ind;

    xref(0, 0) = cx[ind];
    xref(1, 0) = cy[ind];
    xref(2, 0) = cyaw[ind];
    xref(3, 0) = sp[ind];

    float travel = 0.f;
    float nx = xref.rows(); float T = xref.cols();
    for (int i = 0; i < T; ++i ) {
        travel += std::abs(state.v) * dt;

        // compute change in index, add ref point at next horizon
        int dind = static_cast<int>(std::round(travel/dl)); // travel / coursetick
        int n_ind = (ind + dind < ncourse) ? ind + dind : ncourse - 1  ;
        xref(0, i) = cx[n_ind];
        xref(1, i) = cy[n_ind];
        xref(2, i) = cyaw[n_ind];
        xref(3, i) = sp[n_ind];
    }

    target_ind = ind; // NOT NECCESARY?

}




std::vector<float> mpcSolve(BicycleModelRobot& state, Eigen::MatrixXf& xref) {
    double x = static_cast<double>(state.x);
    double y = static_cast<double>(state.y);
    double yaw = static_cast<double>(state.yaw);
    double v = static_cast<double>(state.v);

    size_t T = xref.cols();
    size_t n_vars = T * 4 + (T - 1) * 2;
    size_t n_constraints = T * 4;

    // start position of each state variable
    int x_start = 0;
    int y_start = x_start + T;
    int yaw_start = y_start + T;
    int v_start = yaw_start + T;
    int delta_start = v_start + T;
    int a_start = delta_start + T-1;

    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++)
        vars[i] = 0.0;
    
    vars[x_start] = x;
    vars[y_start] = y;
    vars[yaw_start] = yaw;
    vars[v_start] = v;

    // lower and upper limits of state variable
    // initialize all to min and max of double
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    for (auto i = 0; i < n_vars; i++) {
        vars_lowerbound[i] = std::numeric_limits<double>::min();
        vars_upperbound[i] = std::numeric_limits<double>::max();
    }
    // impose steer, accel and speed constraint
    for (int i = delta_start; i < delta_start+T-1; ++i) {
        vars_lowerbound[i] = -state.max_steer;
        vars_upperbound[i] = state.max_steer;
    }
    for (int i = a_start; i < a_start+T-1; ++i) {
         vars_lowerbound[i] = -state.max_accel;
        vars_upperbound[i] = state.max_accel;
    }
    for (int i = v_start; i < v_start+T; ++i){
        vars_lowerbound[i] = state.min_speed;
        vars_upperbound[i] = state.max_speed;
    }

    // impose constraints
    Dvector constrains_lowerbound(n_constraints);
    Dvector constrains_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; ++i) {
        constrains_lowerbound[i] = 0;
        constrains_upperbound[i] = 0;
    }
    // lowerbound
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[yaw_start] = yaw;
    constraints_lowerbound[v_start] = v;
    // upperbound
    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[yaw_start] = yaw;
    constraints_upperbound[v_start] = v;

    // do optimization
    FG_EVAL fg_eval(traj_ref);

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

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_EVAL>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    bool success = 
        (solution.status == CppAD::ipopt::solve_result<Dvector>::success);


    std::vector<float> res; res.reserve(n_vars);
    for (int i = 0; i < n_vars; ++i) {
        res.push_back(static_cast<float>(solution.x[i]));
    }

    return res;

}


FG_EVAL::FG_EVAL(Eigen::MatrixXf traj_ref, float dt, float L) : 
    traj_ref_(traj_ref), dt_(dt), L_(L) {}


void FG_EVAL::operator()(ADvector& fg, const ADvector& vars) {
    size_t T = traj_ref_.cols();
    // start position of each state variable
    int x_start = 0;
    int y_start = x_start + T;
    int yaw_start = y_start + T;
    int v_start = yaw_start + T;
    int delta_start = v_start + T;
    int a_start = delta_start + T-1;

    // setup fg
    fg[0] = 0;

    // control cost, R
    for (int i = 0; i < T-1; ++i) {
        fg[0] += 0.01 * CppAD::pow(vars[a_start+i], 2);
        fg[0] += 0.01 * CppAD::pow(vars[delta_start+i], 2);
    }

    // control diff cost, Rd
    for(int i=0; i<T-2; i++){
        fg[0] += 0.01 * CppAD::pow(vars[a_start+i+1] - vars[a_start+i], 2);
        fg[0] += 1 * CppAD::pow(vars[delta_start+i+1] - vars[delta_start+i], 2);
    }
    
    // fix the initial state as a constraint
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + yaw_start] = vars[yaw_start];
    fg[1 + v_start] = vars[v_start];

    // state cost, Q
    for (int i = 0; i < T - 1; i++) {
        // state at time t
        AD<double> x0 = vars[x_start + i];
        AD<double> y0 = vars[y_start + i];
        AD<double> yaw0 = vars[yaw_start + i];
        AD<double> v0 = vars[v_start + i];

        // actuation at time t.
        AD<double> delta0 = vars[delta_start + i];
        AD<double> a0 = vars[a_start + i];

        // state at time t+1 .
        AD<double> x1 = vars[x_start + i + 1];
        AD<double> y1 = vars[y_start + i + 1];
        AD<double> yaw1 = vars[yaw_start + i + 1];
        AD<double> v1 = vars[v_start + i + 1];

        // apply dynamic model
        fg[x_start + i + 2] = x1 - (x0 + v0 * CppAD::cos(yaw0) * dt_);
        fg[y_start + i + 2] = y1 - (y0 + v0 * CppAD::sin(yaw0) * dt_);
        fg[yaw_start + i + 2] = yaw1 - (yaw0 + v0 * CppAD::tan(delta0) / L_ * dt_);
        fg[v_start + i + 2] = v1 - (v0 + a0 * dt_);

        // compare with traj and add to cost 
        fg[0] += CppAD::pow(traj_ref_(0, i+1) - (x0 + v0 * CppAD::cos(yaw0) * dt_), 2);
        fg[0] += CppAD::pow(traj_ref_(1, i+1) - (y0 + v0 * CppAD::sin(yaw0) * dt_), 2);
        fg[0] += 0.5 * CppAD::pow(traj_ref_(2, i+1) - (yaw0 + v0 * CppAD::tan(delta0) / L_ * dt_), 2);
        fg[0] += 0.5 * CppAD::pow(traj_ref_(3, i+1) - (v0 + a0 * dt_), 2);
    }

}

