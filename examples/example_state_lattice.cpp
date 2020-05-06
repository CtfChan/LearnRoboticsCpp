#include "state_lattice.hpp"
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

}


void uniformStateSamplingExample2() {
    
}


void biasedStateSamplingExample1() {
    
}


void biasedStateSamplingExample2() {
    
}

void laneStateSamplingExample1() {
    // initialize 
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

    auto [table, paths] = generatePaths(optim, lookup, target_states    , k0);

    Gnuplot gp;

    // gp << "set term gif animate\n";
    // gp << "set output '../animations/model_predictive_trajectory_generation.gif'\n";
    gp << "unset key \n";

    gp << "plot '-' with vectors \n";
    gp.send1d(paths);

    // gp << "set output\n";

}

int main() {



    laneStateSamplingExample1();


    return 0;
}
