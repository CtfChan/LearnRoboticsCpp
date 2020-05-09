#include "path_planning/state_lattice.hpp"
#include "gnuplot-iostream.h"

#include <sstream> 

std::vector<TableEntry> readLookupTable(std::string& file_path) {
    std::vector<TableEntry> table;

    std::ifstream my_file(file_path);

    // Make sure the file is open
    if(!my_file.is_open()) throw std::runtime_error("Could not open file");
    
    std::string line;
    std::string tmp;
    float x, y, yaw, s, km, kf;
    bool first_line = true;
    while (std::getline(my_file, line)) {
        if (first_line) {
            first_line = false;
            continue;
        }

        if (line.size() <= 1)
            continue;

        std::stringstream ss(line);
        std::getline(ss, tmp, ',');
        x = std::stof(tmp);
        std::getline(ss, tmp, ',');
        y = std::stof(tmp);
        std::getline(ss, tmp, ',');
        yaw = std::stof(tmp);
        std::getline(ss, tmp, ',');
        s = std::stof(tmp);
        std::getline(ss, tmp, ',');
        km = std::stof(tmp);
        std::getline(ss, tmp);
        kf = std::stof(tmp);

        TableEntry entry{x, y, yaw, s, km, kf};
        table.push_back(entry);
    }
    
    my_file.close();

    return table;
}


void uniformStateSamplingExample1() {
    float k0 = 0.f;
    size_t nxy = 5;
    size_t nh = 5;
    float d = 20.f;
    float a_min = - deg2rad(45.f);
    float a_max = deg2rad(45.f);
    float p_min = - deg2rad(45.f);
    float p_max = deg2rad(45.f);

    std::vector<Pose2D> target_states = calculateUniformPolarStates( nxy,  nh,  d,
                         a_min,  a_max,  p_min,  p_max);

    // initialize optimizer
    BicycleModelRobot state{0.f, 0.f, 0.f, 3.0f, 1.0f, deg2rad(90.0f)};
    size_t max_iter = 100;
    float cost_th = 0.1f;
    TrajectoryParam delta_param{0.5f, 0.02f, 0.02f};
    float ds = 0.1f; // discretization of path length for trajectory generation
    TrajectoryOptimizer optim(state, max_iter, cost_th, delta_param, ds);

    // read lookuptable
    std::string table_path = "../misc/lookup_table.csv";
    std::vector<TableEntry> lookup = readLookupTable(table_path);

    auto [table, paths] = generatePaths(optim, lookup, target_states, k0);

    Gnuplot gp;
    gp << "unset key \n";
    gp << "plot '-' with vectors \n";
    gp.send1d(paths);
}


void uniformStateSamplingExample2() {
    float k0 = 0.1f;
    size_t nxy = 6;
    size_t nh = 2;
    float d = 20.f;
    float a_min = - deg2rad(-10.f);
    float a_max = deg2rad(45.f);
    float p_min = - deg2rad(20.f);
    float p_max = deg2rad(20.f);

    std::vector<Pose2D> target_states = calculateUniformPolarStates( nxy,  nh,  d,
                         a_min,  a_max,  p_min,  p_max);

    // initialize optimizer
    BicycleModelRobot state{0.f, 0.f, 0.f, 3.0f, 1.0f, deg2rad(90.0f)};
    size_t max_iter = 100;
    float cost_th = 0.1f;
    TrajectoryParam delta_param{0.5f, 0.02f, 0.02f};
    float ds = 0.1f; // discretization of path length for trajectory generation
    TrajectoryOptimizer optim(state, max_iter, cost_th, delta_param, ds);

    // read lookuptable
    std::string table_path = "../misc/lookup_table.csv";
    std::vector<TableEntry> lookup = readLookupTable(table_path);

    auto [table, paths] = generatePaths(optim, lookup, target_states, k0);

    Gnuplot gp;
    gp << "set term png\n";
    gp << "set output '../animations/uniform_sampling.png'\n";
    gp << "unset key \n";
    gp << "plot '-' with vectors \n";
    gp.send1d(paths);
    gp << "set output\n";

}


void biasedStateSamplingExample1() {
    float k0 = 0.f;
    int nxy = 30;
    int nh = 2;
    float d = 20.f;
    float a_min = deg2rad(-45.f);
    float a_max = deg2rad(45.f);
    float p_min = - deg2rad(20.f);
    float p_max = deg2rad(20.f);
    int ns = 100;
    float goal_angle = deg2rad(0.f);

    std::vector<Pose2D> target_states = calculateBiasedPolarStates(
        goal_angle, ns, nxy, nh, d, a_min, a_max, p_min, p_max);

    std::cout  << "size of target_states: " << target_states.size() << std::endl;

    // initialize optimizer
    BicycleModelRobot state{0.f, 0.f, 0.f, 3.0f, 1.0f, deg2rad(90.0f)};
    size_t max_iter = 100;
    float cost_th = 0.1f;
    TrajectoryParam delta_param{0.5f, 0.02f, 0.02f};
    float ds = 0.1f; // discretization of path length for trajectory generation
    TrajectoryOptimizer optim(state, max_iter, cost_th, delta_param, ds);

    // read lookuptable
    std::string table_path = "../misc/lookup_table.csv";
    std::vector<TableEntry> lookup = readLookupTable(table_path);

    auto [table, paths] = generatePaths(optim, lookup, target_states, k0);

    Gnuplot gp;
    gp << "unset key \n";
    gp << "plot '-' with vectors \n";
    gp.send1d(paths);

}


void biasedStateSamplingExample2() {
    float k0 = 0.f;
    size_t nxy = 30;
    size_t nh = 1;
    float d = 20.f;
    float a_min = deg2rad(0.f);
    float a_max = deg2rad(45.f);
    float p_min = - deg2rad(20.f);
    float p_max = deg2rad(20.f);
    size_t ns = 100;
    float goal_angle = deg2rad(30.f);

    std::vector<Pose2D> target_states = calculateBiasedPolarStates(
        goal_angle, ns, nxy, nh, d, a_min, a_max, p_min, p_max);

    // initialize optimizer
    BicycleModelRobot state{0.f, 0.f, 0.f, 3.0f, 1.0f, deg2rad(90.0f)};
    size_t max_iter = 100;
    float cost_th = 0.1f;
    TrajectoryParam delta_param{0.5f, 0.02f, 0.02f};
    float ds = 0.1f; // discretization of path length for trajectory generation
    TrajectoryOptimizer optim(state, max_iter, cost_th, delta_param, ds);

    // read lookuptable
    std::string table_path = "../misc/lookup_table.csv";
    std::vector<TableEntry> lookup = readLookupTable(table_path);

    auto [table, paths] = generatePaths(optim, lookup, target_states, k0);

    Gnuplot gp;
    gp << "set term png\n";

    gp << "set output '../animations/biased_sampling.png'\n";

    gp << "unset key \n";
    gp << "plot '-' with vectors \n";
    gp.send1d(paths);
        gp << "set output\n";

}

void laneStateSamplingExample1() {
    // initialize target states
    float k0 = 0.0;
    float l_center = 10.f;
    float l_heading = deg2rad(90.f);
    float l_width = 3.f;
    float v_width = 1.f;
    float d = 10.f;
    size_t nxy = 5;
    std::vector<Pose2D> target_states = calculateLaneStates(l_center, l_heading, 
                                                l_width, v_width, d, nxy);

    // initialize optimizer
    BicycleModelRobot state{0.f, 0.f, 0.f, 3.0f, 1.0f, deg2rad(90.0f)};
    size_t max_iter = 100;
    float cost_th = 0.1f;
    TrajectoryParam delta_param{0.5f, 0.02f, 0.02f};
    float ds = 0.1f; // discretization of path length for trajectory generation
    TrajectoryOptimizer optim(state, max_iter, cost_th, delta_param, ds);


    // read lookuptable
    std::string table_path = "../misc/lookup_table.csv";
    std::vector<TableEntry> lookup = readLookupTable(table_path);

    // generate paths
    auto [table, paths] = generatePaths(optim, lookup, target_states    , k0);

    // do plottings
    Gnuplot gp;
    gp << "set term png\n";

    gp << "set output '../animations/lane_sampling.png'\n";

    gp << "unset key \n";
    gp << "plot '-' with vectors \n";
    gp.send1d(paths);
        gp << "set output\n";

}

int main() {


    // uniformStateSamplingExample1();
    uniformStateSamplingExample2();

    // biasedStateSamplingExample1();
    biasedStateSamplingExample2();

    laneStateSamplingExample1();


    return 0;
}
