# include "quintic_polynomial.hpp"

# include "common.hpp"

# include "gnuplot-iostream.h"





int main() {
    // (x, y, yaw, v, a)
    State start{10.f, 10.f, deg2rad(10.f), 1.f, 0.1f};
    State goal{30.f, -10.f, deg2rad(20.f), 1.f, 0.1f};

    float max_accel = 1.0f;
    float max_jerk = 0.5f;
    float dt = 0.1;

    Gnuplot gp;




}