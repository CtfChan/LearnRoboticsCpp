#include "path_planning/quintic_polynomial.hpp"
#include "common.hpp"


#include<Eigen/Eigen>

QuinticPolynomial::QuinticPolynomial(float xs, float vs, float as, 
    float xe, float ve, float ae, float T) {
    // solve for a0, a1, a2
    a0_ = xs;
    a1_ = vs;
    a2_ = as / 2.f;

    // solve for x = (a3, a4, a5)^T
    Eigen::Matrix3f A;
    A << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5),
        3*std::pow(T, 2), 4*std::pow(T, 3), 5*std::pow(T,4),
        6*T, 12 * std::pow(T, 2), 20*std::pow(T,3);
    Eigen::Vector3f b;
    b << xe - a0_ - a1_ * T - a2_ * std::pow(T, 2),
         ve - a1_ - 2 * a2_ * T,
         ae - 2 * a2_;
    Eigen::Vector3f x = A.colPivHouseholderQr().solve(b);

    a3_ = x(0);
    a4_ = x(1);
    a5_ = x(2);
}

float QuinticPolynomial::calculatePoint(float t) {
    return a0_+ a1_*t + a2_*std::pow(t, 2) + 
           a3_*std::pow(t, 3) + a4_*std::pow(t, 4) + 
           a5_*std::pow(t, 5);
}

float QuinticPolynomial::calculateFirstDerivative(float t) {
    return a1_ + 2*a2_*t + 3*a3_*std::pow(t, 2) + 
            4*a4_*std::pow(t, 3) + 5*a5_*std::pow(t, 4);
}

float QuinticPolynomial::calculateSecondDerivative(float t) {
    return 2*a2_ + 6*a3_*t + 12*a4_*std::pow(t, 2) + 20*a5_*std::pow(t, 3);
}

float QuinticPolynomial::calculateThirdDerivative(float t) {
    return 6*a3_ + 24*a4_*t + 60*a5_*std::pow(t,2);
}




QuinticPolynomialTrajectory quinticPolynomialPlanner(State& start, State& goal,
    float min_T, float max_T, float T_res, float max_accel, float max_jerk, float dt) {

    // determine velocity and acceleration along x,y direction for start and goal
    float vxs = start.v * std::cos(start.yaw);
    float vys = start.v * std::sin(start.yaw);
    float vxg = goal.v * std::cos(goal.yaw);
    float vyg = goal.v * std::sin(goal.yaw);

    float axs = start.a * std::cos(start.yaw);
    float ays = start.a * std::sin(start.yaw);
    float axg = goal.a * std::cos(goal.yaw);
    float ayg = goal.a * std::sin(goal.yaw);

    std::vector<float> time, rx, ry, ryaw, rv, ra, rj;

    // try all possible Ts
    for (float T = min_T; T < max_T; T += T_res) {
        auto xqp = QuinticPolynomial(start.x, vxs, axs, goal.x, vxg, axg, T);
        auto yqp = QuinticPolynomial(start.y, vys, ays, goal.y, vyg, ayg, T);

        time.clear(); rx.clear(); ry.clear(); ryaw.clear(); rv.clear(); ra.clear(); rj.clear();
        bool valid_path = true;

        // determine if max_accel and max_jerk constr are satisfied
        for (float t = 0.0f; t < T + dt; t += dt) {
            time.push_back(t);
            float x = xqp.calculatePoint(t);
            float y = yqp.calculatePoint(t);
            rx.push_back(x);
            ry.push_back(y);

            float vx = xqp.calculateFirstDerivative(t);
            float vy = yqp.calculateFirstDerivative(t);
            float v = std::hypot(vx, vy);
            float yaw = std::atan2(vy, vx);
            ryaw.push_back(yaw);
            rv.push_back(v);

            float ax = xqp.calculateSecondDerivative(t);
            float ay = yqp.calculateSecondDerivative(t);
            float a = std::hypot(ax, ay);
            if (rv.size() >= 2 && rv.end()[-1] - rv.end()[-2] < 0.f) 
                a *= -1; // decceleration, add sign
            ra.push_back(a);

            float jx = xqp.calculateThirdDerivative(t);
            float jy = yqp.calculateThirdDerivative(t);
            float j = std::hypot(jx, jy);
            if (rj.size() >= 2 && ra.end()[-1] - ra.end()[-2] < 0.f)
                j *= -1; // reduction in jerk, add sign
            rj.push_back(j);

            if (std::abs(a) > max_accel || std::abs(j) > max_jerk) {
                valid_path = false;
                break;
            }
        }

        // return first valid path you see
        if (valid_path) {
            return {time, rx, ry, ryaw, rv, ra, rj};
        }
    }

    return {};
}


// int main() {
//     // (x, y, yaw, v, a)
//     State start{10.f, 10.f, deg2rad(10.f), 1.f, 0.1f};
//     State goal{30.f, -10.f, deg2rad(20.f), 1.f, 0.1f};

//     float max_accel = 1.0f;
//     float max_jerk = 0.5f;
//     float dt = 0.1;

//     // min and max time to goal
//     float max_T = 100.f;
//     float min_T = 5.f;
//     float T_res = 5.f;

//     Gnuplot gp;

//     gp << "set term gif animate\n";
//     gp << "set output '../animations/quintic_polynomial.gif'\n";
//     gp << "set size 1,1 \n";

//     QuinticPolynomialTrajectory states = quinticPolynomialPlanner(
//         start, goal, min_T, max_T, T_res, max_accel, max_jerk, dt);
    
//     // generate animation
//     for (size_t i = 0; i < states.time.size(); ++i) {

//         gp << "set multiplot\n";

//         std::vector<float> plot_x(states.x.begin(), states.x.begin() + i);
//         std::vector<float> plot_y(states.y.begin(), states.y.begin() + i);
//         std::vector<float> plot_time(states.time.begin(), states.time.begin() + i);
//         std::vector<float> plot_v(states.v.begin(), states.v.begin() + i);
//         std::vector<float> plot_a(states.a.begin(), states.a.begin() + i);
//         std::vector<float> plot_j(states.j.begin(), states.j.begin() + i);

//         // plot trajectory
//         gp << "set size 0.5, 0.5\n";
//         gp << "set origin 0.0, 0.5\n"; // origin is lower left corner of graph
//         gp << "set xrange [0:35]\nset yrange [-20:15]\n";
//         gp << "plot '-' with line title 'path',"
//               "'-' with vector title 'pose' lw 3 ,"
//               "'-' with vector title 'start' lw 2,"
//               "'-' with vector title 'goal' lw 2\n";
        
//         gp.send1d(boost::make_tuple(plot_x, plot_y));
//         gp.send1d(poseToVector(states.x[i], states.y[i], states.yaw[i], 0.5));
//         gp.send1d(poseToVector(start.x, start.y, start.yaw, 0.5));
//         gp.send1d(poseToVector(goal.x, goal.y, goal.yaw, 0.5));

//         // plot velocity
//         gp << "set size 0.5, 0.5\n";
//         gp << "set origin 0.5, 0.5\n"; 
//         gp << "set xrange [0:50]\nset yrange [-5:5]\n";
//         gp << "plot '-' with line title 'velocity'\n";
//         gp.send1d(boost::make_tuple(plot_time, plot_v));

//         // plot acc
//         gp << "set size 0.5, 0.5\n";
//         gp << "set origin 0.0, 0.0\n"; 
//         gp << "set xrange [0:50]\nset yrange [-1:1]\n";
//         gp << "plot '-' with line title 'acceleration'\n";
//         gp.send1d(boost::make_tuple(plot_time, plot_a));

//         // plot jerk
//         gp << "set size 0.5, 0.5\n";
//         gp << "set origin 0.5, 0.0\n"; 
//         gp << "set xrange [0:50]\nset yrange [-1:1]\n";
//         gp << "plot '-' with line title 'jerk'\n";
//         gp.send1d(boost::make_tuple(plot_time, plot_j));


//         gp << "unset multiplot\n";

//     }
//     gp << "set output\n";

// }