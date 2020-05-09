

#include "path_planning/cubic_spline.hpp"
#include "common.hpp"
#include "path_tracking/mpc.hpp"
#include "gnuplot-iostream.h"



#include <iostream>
#include <iomanip>
#include <limits>
#include <vector>

#include <Eigen/Eigen>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include <algorithm>

#define NX 4
#define T 6

#define DT 0.2
#define MAX_STEER 45.0 / 180 * M_PI

#define N_IND_SEARCH 10
#define MAX_TIME 500

#define WB 2.5
#define MAX_SPEED 55.0 / 3.6
#define MIN_SPEED -20.0 / 3.6
#define MAX_ACCEL 1.0


using CppAD::AD;
using M_XREF = Eigen::Matrix<float, NX, T>;

int x_start = 0;
int y_start = x_start + T;
int yaw_start = y_start + T;
int v_start = yaw_start + T;

int delta_start = v_start + T;
int a_start = delta_start + T - 1;



std::vector<float> calculateSpeedProfile(std::vector<float> rx, std::vector<float> ry, std::vector<float> ryaw, float target_speed)
{
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
};

int calculateNearestIndex(BicycleModelRobot state, std::vector<float> cx, std::vector<float> cy, std::vector<float> cyaw, int pind)
{
    float mind = std::numeric_limits<float>::max();
    float ind = 0;
    for (unsigned int i = pind; i < pind + N_IND_SEARCH; i++)
    {
        float idx = cx[i] - state.x;
        float idy = cy[i] - state.y;
        float d_e = idx * idx + idy * idy;

        if (d_e < mind)
        {
            mind = d_e;
            ind = i;
        }
    }

    return ind;
};

void calculateReferenceTrajectory(BicycleModelRobot state, std::vector<float> cx, std::vector<float> cy,
                         std::vector<float> cyaw, std::vector<float> ck, std::vector<float> sp, float dl, int &target_ind, M_XREF &xref)
{

    xref = M_XREF::Zero();

    int ncourse = cx.size();

    int ind = calculateNearestIndex(state, cx, cy, cyaw, target_ind);
    if (target_ind >= ind)
        ind = target_ind;

    xref(0, 0) = cx[ind];
    xref(1, 0) = cy[ind];
    xref(2, 0) = cyaw[ind];
    xref(3, 0) = sp[ind];

    float travel = 0.0;

    for (int i = 0; i < T; i++)
    {
        travel += std::abs(state.v) * DT;
        int dind = static_cast<int>(  std::round(travel / dl)      );
        int new_ind = (ind + dind < ncourse) ? ind + dind : ncourse;

        xref(0, i) = cx[new_ind];
        xref(1, i) = cy[new_ind];
        xref(2, i) = cyaw[new_ind];
        xref(3, i) = sp[new_ind];
    }

    target_ind = ind;
};

void smoothYaw(std::vector<float> &cyaw)
{
    for (unsigned int i = 0; i < cyaw.size() - 1; i++)
    {
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
};

class FG_EVAL
{
public:
    M_XREF traj_ref;

    FG_EVAL(M_XREF traj_ref)
    {
        this->traj_ref = traj_ref;
    }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars)
    {
        fg[0] = 0;

        for (int i = 0; i < T - 1; i++)
        {
            fg[0] += 0.01 * CppAD::pow(vars[a_start + i], 2);
            fg[0] += 0.01 * CppAD::pow(vars[delta_start + i], 2);
        }

        for (int i = 0; i < T - 2; i++)
        {
            fg[0] += 0.01 * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
            fg[0] += 1 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
        }

        // fix the initial state as a constraint
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + yaw_start] = vars[yaw_start];
        fg[1 + v_start] = vars[v_start];

        // The rest of the constraints
        for (int i = 0; i < T - 1; i++)
        {
            // The state at time t+1 .
            AD<double> x1 = vars[x_start + i + 1];
            AD<double> y1 = vars[y_start + i + 1];
            AD<double> yaw1 = vars[yaw_start + i + 1];
            AD<double> v1 = vars[v_start + i + 1];

            // The state at time t.
            AD<double> x0 = vars[x_start + i];
            AD<double> y0 = vars[y_start + i];
            AD<double> yaw0 = vars[yaw_start + i];
            AD<double> v0 = vars[v_start + i];

            // Only consider the actuation at time t.
            AD<double> delta0 = vars[delta_start + i];
            AD<double> a0 = vars[a_start + i];

            // constraint with the dynamic model
            fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(yaw0) * DT);
            fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(yaw0) * DT);
            fg[2 + yaw_start + i] = yaw1 - (yaw0 + v0 * CppAD::tan(delta0) / WB * DT);
            fg[2 + v_start + i] = v1 - (v0 + a0 * DT);
            // cost with the ref traj
            fg[0] += CppAD::pow(traj_ref(0, i + 1) - (x0 + v0 * CppAD::cos(yaw0) * DT), 2);
            fg[0] += CppAD::pow(traj_ref(1, i + 1) - (y0 + v0 * CppAD::sin(yaw0) * DT), 2);
            fg[0] += 0.5 * CppAD::pow(traj_ref(2, i + 1) - (yaw0 + v0 * CppAD::tan(delta0) / WB * DT), 2);
            fg[0] += 0.5 * CppAD::pow(traj_ref(3, i + 1) - (v0 + a0 * DT), 2);
        }
    }
};

std::vector<float> mpcSolve(BicycleModelRobot x0, M_XREF traj_ref)
{

    typedef CPPAD_TESTVECTOR(double) Dvector;
    double x = x0.x;
    double y = x0.y;
    double yaw = x0.yaw;
    double v = x0.v;

    size_t n_vars = T * 4 + (T - 1) * 2;
    size_t n_constraints = T * 4;

    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++)
    {
        vars[i] = 0.0;
    }

    vars[x_start] = x;
    vars[y_start] = y;
    vars[yaw_start] = yaw;
    vars[v_start] = v;

    // Lower and upper limits for x
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    // NOTE there mush be both lower and upper bounds for all vars!!!!!
    for (auto i = 0; i < n_vars; i++)
    {
        vars_lowerbound[i] = -10000000.0;
        vars_upperbound[i] = 10000000.0;
    }

    for (auto i = delta_start; i < delta_start + T - 1; i++)
    {
        vars_lowerbound[i] = -x0.max_steer;
        vars_upperbound[i] = x0.max_steer;
    }

    for (auto i = a_start; i < a_start + T - 1; i++)
    {
        vars_lowerbound[i] = -x0.max_accel;
        vars_upperbound[i] = x0.max_accel;
    }

    for (auto i = v_start; i < v_start + T; i++)
    {
        vars_lowerbound[i] = x0.min_speed;
        vars_upperbound[i] = x0.max_speed;
    }

    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (auto i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[yaw_start] = yaw;
    constraints_lowerbound[v_start] = v;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[yaw_start] = yaw;
    constraints_upperbound[v_start] = v;

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

    bool ok = (solution.status == CppAD::ipopt::solve_result<Dvector>::success);

    std::cout << "Optimization success?: " << ok << std::endl;



    std::vector<float> result;
    for (auto i = 0; i < n_vars; i++)
    {
        result.push_back((float)solution.x[i]);
    }
    return result;
};

int main()
{
    // setup plotting
    Gnuplot gp;
    std::vector<float> x_h; // trajectory
    std::vector<float> y_h;
    std::vector<float> x_ref; // tracking/reference point
    std::vector<float> y_ref;

    gp << "set size 1,1 \n";
    gp << "set term gif animate\n";
    gp << "set output '../animations/mpc.gif'\n";


    // generate spline
    std::vector<float> wx({0.0, 60.0, 125.0, 50.0, 75.0, 35.0, -10.0});
    std::vector<float> wy({0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0});
    float ds = 0.1;
    auto [cx, cy, cyaw, ck, cs] = calculateSplineCourse(wx, wy, ds);

    // setup velocity profile
    float target_speed = 10.0 / 3.6;
    std::vector<float> speed_profile = calculateSpeedProfile(cx, cy, cyaw, target_speed);

    // define goal and goal thresh
    float goal_x = cx.back(), goal_y = cy.back();
    float goal_dis = 0.5;

    // define state
    BicycleModelRobot state(cx[0], cy[0], cyaw[0], speed_profile[0], 
        WB, MAX_STEER, MIN_SPEED, MAX_SPEED, MAX_ACCEL, false);

    // calculate idx to track
    int target_ind = 0;
    calculateNearestIndex(state, cx, cy, cyaw, target_ind);

    // smooth out yaw
    smoothYaw(cyaw);


    M_XREF xref;


    float time = 0.0;
    while (MAX_TIME >= time)
    {
        calculateReferenceTrajectory(state, cx, cy, cyaw, ck, speed_profile, 1.0, target_ind, xref);

        std::vector<float> output = mpcSolve(state, xref);
        float acc = output[a_start];
        float steer = output[delta_start];
        state.update(acc, steer, DT);


        float dx = state.x - goal_x;
        float dy = state.y - goal_y;
        if (std::hypot(dx, dy) <= goal_dis)
        {
            std::cout << ("Goal") << std::endl;
            break;
        }

        x_h.push_back(state.x);
        y_h.push_back(state.y);


        // setup ref
        x_ref.clear(); y_ref.clear();
        x_ref.push_back(xref(0, 0));
        y_ref.push_back(xref(1, 0));


        // plot
        gp << "plot '-' with lines title 'path',"
              "'-' title 'actual trajectory',"
              "'-' title 'reference' pointtype 13 pointsize 3\n";
        gp.send1d(boost::make_tuple(cx, cy));
        gp.send1d(boost::make_tuple(x_h, y_h));
        gp.send1d(boost::make_tuple(x_ref, y_ref));

        time += DT;
    }

    gp << "set output\n";

    return 0;
}
