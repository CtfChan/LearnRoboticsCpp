

#include "path_planning/cubic_spline.hpp"
#include "common.hpp"
#include "path_tracking/mpc.hpp"
#include "gnuplot-iostream.h"



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


    // define state
    float wb = 2.5, max_steer = deg2rad(45.f), min_speed = -20.f/3.6f, max_speed = 55.f/3.6f, max_accel = 1.f;
    BicycleModelRobot state(cx[0], cy[0], cyaw[0], speed_profile[0], 
        wb, max_steer, min_speed, max_speed, max_accel, false);

    // mpc controller
    int horizon = 6;
    int n_ind_search = 10;
    float dt = 0.2f;
    float goal_dist = 0.5f;
    ModelPredictiveController mpc(horizon, n_ind_search, dt, goal_dist);
    mpc.setPath(cx, cy, cyaw, ck, speed_profile);

    float time = 0.0f;
    float max_time = 500.f;
    while (max_time >= time)
    {
        auto [acc, steer] = mpc.control(state);
        state.update(acc, steer, dt);

        if (mpc.atGoal(state)) {
            std::cout << ("Goal") << std::endl;
            break;
        }

        x_h.push_back(state.x);
        y_h.push_back(state.y);

        // setup ref
        x_ref.clear(); y_ref.clear();
        auto [ref_pt_x, ref_pt_y] = mpc.getReferencePosition();
        x_ref.push_back(ref_pt_x);
        y_ref.push_back(ref_pt_y);

        // plot
        gp << "plot '-' with lines title 'path',"
              "'-' title 'actual trajectory',"
              "'-' title 'reference' pointtype 13 pointsize 3\n";
        gp.send1d(boost::make_tuple(cx, cy));
        gp.send1d(boost::make_tuple(x_h, y_h));
        gp.send1d(boost::make_tuple(x_ref, y_ref));

        time += dt;
    }

    gp << "set output\n";

    return 0;
}





















