
#include "path_tracking/mpc.hpp"
#include "cubic_spline.hpp"
#include "common.hpp"
// #include <cppad/cppad.hpp>
// #include <cppad/ipopt/solve.hpp>

// using CppAD::AD;


// typedef CPPAD_TESTVECTOR(double) Dvector;




int main() {
    // sim params
    float max_time = 400.f;
    float dt = 0.2f;
    float time = 0.f;

    // define spline
    std::vector<float> ax = {0.0f, 60.0f, 125.0f,  50.0f,   75.0f,  35.0f,  -10.0f};
    std::vector<float> ay = {0.0f,  0.0f,  50.0f,  65.0f,   30.0f,  50.0f,  -20.0f};
    float ds = 0.1f;
    auto [cx, cy, cyaw, ck, s] = calculateSplineCourse(ax, ay, ds);

    // calc speed profile
    float target_speed = 10.0f / 3.6f;
    std::vector<float> vel_prof = calculateSpeedProfile(cx, cy, cyaw, target_speed);

    float L = 2.5f; //m
    float max_steer = deg2rad(45.f);
    float max_speed = 55.f / 3.6f;
    float min_speed = -20.f / 3.6f;
    float max_accel = 1.f;
    BicycleModelRobot state{cx[0], cy[0], cyaw[0], vel_prof[0], 
                        L, max_steer, min_speed, max_speed, max_accel};

    // smooth yaws, ensure delta between yaws is acute
    smoothYaw(cyaw);

    
    int nx = 4; // number of states
    int T = 6; // time horizon
    Eigen::Matrix<float, nx, T> xref;

    // calc target idx
    int n_ind = 10;
    int ind = calculateNearestIndex(state, cx, cy, cyaw, 0);


    // while sim_time
    while (time <= max_time) {
         // calc. ref. traj.

        // do mpc

        // update state

        // save state


        // plot

        // update time
        time += dt;
    }

       



    return 0;
}