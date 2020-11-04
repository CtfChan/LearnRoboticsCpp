

#include "path_planning/state_lattice.hpp"

#include <numeric>


std::vector<Pose2D> sampleStates(std::vector<float>& angle_samples, float a_min,
                 float a_max, float d, float p_max, float p_min, size_t nh) {
    std::vector<Pose2D> states;

    float xf, yf, yawf;
    for (float i : angle_samples) {
        float a = a_min + (a_max - a_min) * i;
        for (int j = 0; j < nh; ++j) {
            xf = d * std::cos(a);
            yf = d * std::sin(a);
            if (nh == 1)
                yawf = (p_max - p_min) / 2.f + a;
            else
                yawf = p_min + (p_max - p_min) * j / (nh - 1.f) + a;
            Pose2D final_state {xf, yf, yawf};
            states.push_back(final_state);
        }
    }

    return states;
}


std::vector<Pose2D> calculateUniformPolarStates(size_t nxy, size_t nh, float d,
                        float a_min, float a_max, float p_min, float p_max) {

    std::vector<float> angle_samples(nxy);
    for (int i = 0; i < nxy; ++i)
        angle_samples[i] = i / (nxy - 1.f);

    std::vector<Pose2D> states = sampleStates(angle_samples, a_min, a_max, d, p_max, p_min, nh);

    return states;
}


std::vector<Pose2D> calculateBiasedPolarStates(float goal_angle, int ns, int nxy,
                                   int nh, int d,
                                   float a_min, float a_max,
                                   float p_min, float p_max){
    std::vector<float> as(ns-1);
    std::vector<float> cnav(ns-1);
    for (size_t i = 0; i < ns - 1; ++i) {
        as[i] = a_min + (a_max - a_min) * i / (ns - 1.f);
        cnav[i] = M_PI - std::abs(as[i] - goal_angle);
    }

    float cnav_sum = std::accumulate(cnav.begin(), cnav.end(), 0.f);
    float cnav_max = *std::max_element(cnav.begin(), cnav.end());

    // normalise
    for (float& cnavi : cnav) {
        cnavi = (cnav_max - cnavi) / (cnav_max * ns - cnav_sum);
    }

    // cumalative sum
    std::vector<float> cumsum_cnav(cnav.size());
    std::partial_sum(cnav.begin(), cnav.end(), cumsum_cnav.begin());

    // output angles
    std::vector<float> di;
    int li = 0;
    for (int i = 0; i < nxy; ++i ) {
        for (int ii = li; ii < ns-1; ++ii) {
            if ( ii*(1.f/ ns) >= i / (nxy - 1.f) ) {
                di.push_back(cumsum_cnav[ii]);
                li = ii - 1;
                break;
            }
        }
    }

    auto states = sampleStates(di, a_min, a_max, d, p_max, p_min, nh);
    return states;
}



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

