#include "path_planning/quintic_polynomial.hpp"
#include "gnuplot-iostream.h"
#include "common.hpp"


int main() {
    // (x, y, yaw, v, a)
    State start{10.f, 10.f, deg2rad(10.f), 1.f, 0.1f};
    State goal{30.f, -10.f, deg2rad(20.f), 1.f, 0.1f};

    float max_accel = 1.0f;
    float max_jerk = 0.5f;
    float dt = 0.1;

    // min and max time to goal
    float max_T = 100.f;
    float min_T = 5.f;
    float T_res = 5.f;

    Gnuplot gp;

    gp << "set term gif animate\n";
    gp << "set output '../animations/quintic_polynomial.gif'\n";
    gp << "set size 1,1 \n";

    QuinticPolynomialTrajectory states = quinticPolynomialPlanner(
        start, goal, min_T, max_T, T_res, max_accel, max_jerk, dt);
    
    // generate animation
    for (size_t i = 0; i < states.time.size(); ++i) {

        gp << "set multiplot\n";

        std::vector<float> plot_x(states.x.begin(), states.x.begin() + i);
        std::vector<float> plot_y(states.y.begin(), states.y.begin() + i);
        std::vector<float> plot_time(states.time.begin(), states.time.begin() + i);
        std::vector<float> plot_v(states.v.begin(), states.v.begin() + i);
        std::vector<float> plot_a(states.a.begin(), states.a.begin() + i);
        std::vector<float> plot_j(states.j.begin(), states.j.begin() + i);

        // plot trajectory
        gp << "set size 0.5, 0.5\n";
        gp << "set origin 0.0, 0.5\n"; // origin is lower left corner of graph
        gp << "set xrange [0:35]\nset yrange [-20:15]\n";
        gp << "plot '-' with line title 'path',"
              "'-' with vector title 'pose' lw 3 ,"
              "'-' with vector title 'start' lw 2,"
              "'-' with vector title 'goal' lw 2\n";
        
        gp.send1d(boost::make_tuple(plot_x, plot_y));
        gp.send1d(poseToVector(states.x[i], states.y[i], states.yaw[i], 0.5));
        gp.send1d(poseToVector(start.x, start.y, start.yaw, 0.5));
        gp.send1d(poseToVector(goal.x, goal.y, goal.yaw, 0.5));

        // plot velocity
        gp << "set size 0.5, 0.5\n";
        gp << "set origin 0.5, 0.5\n"; 
        gp << "set xrange [0:50]\nset yrange [-5:5]\n";
        gp << "plot '-' with line title 'velocity'\n";
        gp.send1d(boost::make_tuple(plot_time, plot_v));

        // plot acc
        gp << "set size 0.5, 0.5\n";
        gp << "set origin 0.0, 0.0\n"; 
        gp << "set xrange [0:50]\nset yrange [-1:1]\n";
        gp << "plot '-' with line title 'acceleration'\n";
        gp.send1d(boost::make_tuple(plot_time, plot_a));

        // plot jerk
        gp << "set size 0.5, 0.5\n";
        gp << "set origin 0.5, 0.0\n"; 
        gp << "set xrange [0:50]\nset yrange [-1:1]\n";
        gp << "plot '-' with line title 'jerk'\n";
        gp.send1d(boost::make_tuple(plot_time, plot_j));


        gp << "unset multiplot\n";

    }
    gp << "set output\n";

}