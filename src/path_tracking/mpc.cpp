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



}