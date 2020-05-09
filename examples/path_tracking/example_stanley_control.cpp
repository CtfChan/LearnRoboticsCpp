#include "path_tracking/stanley_control.hpp"
#include "path_planning/cubic_spline.hpp"

#include "gnuplot-iostream.h"

int main() {
    std::vector<float> ax = {0.f, 100.f, 100.f, 50.f, 60.f};
    std::vector<float> ay = {0.f, 0.f, -30.f, -20.f, 0.0f};
    float ds = 0.1f;

    auto [cx, cy, cyaw, ck, s] = calculateSplineCourse(ax, ay, ds);

    // Initial state
    float xs = 0.f, ys = 5.f, yaws = deg2rad(20.f), 
        vs = 0.f, L = 2.9f, max_steer = deg2rad(30.f);
    BicycleModelRobot state{xs, ys, yaws, vs, L, max_steer};

    float k = 0.5f; // control gain
    float Kp = 1.0f; // speed proportional gain
    float dt = 0.1f;
    float target_speed = 30.0f / 3.6f ; // [m/s]

    float max_sim_time = 100.0f;
    float time = 0.f;

    StanleyController controller(k, Kp);
    controller.setPath(cx, cy, cyaw, target_speed);
    
    Gnuplot gp;
    gp << "set size 1,1 \n";
    gp << "set term gif animate\n";
    gp << "set output '../animations/stanley_control.gif'\n";

    std::vector<float> x, y, yaw, v, t;
    while (max_sim_time >= time && controller.hasArrived() == false) {
        auto [acc, delta] = controller.computeControl(state);
        state.update(acc, delta, dt);
        time += dt;
        
        x.push_back(state.x);
        y.push_back(state.y);
        yaw.push_back(state.yaw);
        v.push_back(state.v);
        t.push_back(time);

        gp << "set multiplot\n";

        // plot trajectory
        gp << "set size 1, 0.5\n";
        gp << "set origin 0.0, 0.0\n"; // origin is lower left corner of graph
        gp << "set title 'Trajectory'\n";
        gp << "plot '-' with line title 'target trajectory',"
                   "'-' with line title 'actual trajectory',"
                   "'-' with vectors title 'robot' lw 3\n";
        gp.send1d(boost::make_tuple(cx, cy));
        gp.send1d(boost::make_tuple(x, y));
        gp.send1d(poseToVector(state.x, state.y, state.yaw, state.L));

        // plot velocity
        gp << "set size 1, 0.5\n";
        gp << "set origin 0.0, 0.5\n"; // origin is lower left corner of graph
        gp << "set title 'Velocity vs. Time'\n";
        gp << "plot '-' with line\n";
        gp.send1d(boost::make_tuple(t, v ));

        // sleep(1);

        gp << "unset multiplot\n";
    }

    gp << "set output\n";

    return 0;
}   