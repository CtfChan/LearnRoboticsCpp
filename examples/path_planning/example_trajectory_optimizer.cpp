#include "common.hpp"
#include "path_planning/trajectory_optimizer.hpp"

int main() {
    Gnuplot gp;

    // trajectory is robot centric, so start of optimization always at (0, 0, 0)
    // (x, y, yaw, v, wheel_base, max_steer)
    BicycleModelRobot state{0.f, 0.f, 0.f, 3.0f, 1.0f, deg2rad(90.0f)};

    // initialize optimizer
    size_t max_iter = 100;
    float cost_th = 0.1f;
    TrajectoryParam delta_param{0.5f, 0.02f, 0.02f};
    float ds = 0.1f;  // discretization of path length for trajectory generation
    TrajectoryOptimizer optim(state, max_iter, cost_th, delta_param, ds);

    // do optimization
    Pose2D target_pose{5.0f, 2.0f, deg2rad(90.0f)};
    TrajectoryParam init_param{
        6.0f, 0.0f, 0.0f};  // p = (path len, middle_steer, final_steer)
    float k0 = 0.0f;        // initial steer
    Pose2DTrajectory traj =
        optim.optimizeTrajectory(target_pose, init_param, k0, gp);

    return 0;
}