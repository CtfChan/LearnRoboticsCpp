#include "stanley_control.hpp"
#include "cubic_spline.hpp"

int main() {
    std::vector<float> ax = {0.f, 100.f, 100.f, 50.f, 60.f};
    std::vector<float> ay = {0.f, 0.f, -30.f, -20.f, 0.0f};
    float ds = 0.1f;

    auto [cx, cy, cyaw, ck, s] = calculateSplineCourse(ax, ay, ds);

    // Initial state
    float x = 0.f, y = 5.f, yaw = deg2rad(20.f), 
        v = 0.f, L = 2.9f, max_steer = deg2rad(30.f);
    BicycleModelRobot state{x, y, yaw, v, L, max_steer};

    float k = 0.5f; // control gain
    float Kp = 1.0f; // speed proportional gain
    float dt = 0.1f;
    float target_speed = 30.0f / 3.6f ; // [m/s]

    float max_sim_time = 100.0f;



    return 0;
}   