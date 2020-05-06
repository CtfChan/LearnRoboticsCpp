

#include "state_lattice.hpp"

std::vector<Pose2D> calculateLaneStates(
    float l_center, float l_heading, float l_width, 
    float v_width, float d, size_t nxy) {
    float xc = d * std::cos(l_heading) + l_center * std::sin(l_heading);
    float yc = d * std::sin(l_heading) + l_center * std::cos(l_heading);

    std::vector<Pose2D> states; states.reserve(nxy);
    for (size_t i = 0; i < nxy; ++i) {
        float delta = -0.5*(l_width-v_width) + (l_width-v_width) * (i / (nxy - 1.f));
        float xf = xc - delta * std::sin(l_heading);
        float yf = yc + delta * std::cos(l_heading);
        float yawf = l_heading;
        Pose2D target_state{xf, yf, yawf};
        states.push_back(target_state);
    }

    return states;
}


std::pair<std::vector<TableEntry>, Arrow> generatePaths(TrajectoryOptimizer& optim, std::vector<TableEntry>& lookup,
                                     std::vector<Pose2D>& states, float k0) {
    
    // track all valid end points states
    std::vector<TableEntry> table;

    // for visualization of paths
    Arrow all_arr;

    // iterate through all possible final states
    for (auto& state : states) {
        TableEntry entry = findClosestEntry(lookup, state.x, state.y, state.theta);

        // use closest entry as initial guess, p = (path len, middle_steer, final_steer)
        TrajectoryParam param{ entry.s, entry.km, entry.kf};
        // get trajectory
        Pose2DTrajectory traj = optim.optimizeTrajectory(state, param, k0);

        if (!traj.x.empty()) {
            TableEntry new_entry{state.x, state.y, state.theta, param.s, param.km, param.kf};
            table.push_back(new_entry);

            // generate plot
            Arrow arr = trajectoryToVector(traj);
            all_arr.insert(all_arr.end(), arr.begin(), arr.end());
        }
    }


    return {table, all_arr};
}

// // trajectory is robot centric, so start of optimization always at (0, 0, 0)
//     // (x, y, yaw, v, wheel_base, max_steer)
//     BicycleModelRobot state{0.f, 0.f, 0.f, 3.0f, 1.0f, deg2rad(90.0f)};
//     float k0 = 0.0f; // initial steer

//     // initialize optimizer
//     size_t max_iter = 100;
//     float cost_th = 0.1f;
//     TrajectoryParam delta_param{0.5f, 0.02f, 0.02f};
//     float ds = 0.1f; // discretization of path length for trajectory generation
//     TrajectoryOptimizer optim(state, max_iter, cost_th, delta_param, ds);

