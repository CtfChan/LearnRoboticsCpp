
#include "trajectory_optimizer.hpp"
#include "common.hpp"

struct TableEntry {
    float x;
    float y;
    float yaw;
    float s;
    float km;
    float kf;
};


TableEntry findClosestEntry(const std::vector<TableEntry>& table, float x, float y, float yaw) {
    auto closest = std::min_element(table.begin(), table.end(), [&]
    (const auto& e1, const auto& e2) {
        float dx1 = e1.x - x, dy1 = e1.y - y, dyaw1 = e1.yaw - yaw;
        float dx2 = e2.x - x, dy2 = e2.y - y, dyaw2 = e2.yaw - yaw;
        float c1 = std::pow(dx1, 2) + std::pow(dy1, 2) + std::pow(dyaw1, 2);
        float c2 = std::pow(dx2, 2) + std::pow(dy2, 2) + std::pow(dyaw2, 2);
        return c1 < c2;
    });

    return *closest;

}

int main() {

    // trajectory is robot centric, so start of optimization always at (0, 0, 0)
    // (x, y, yaw, v, wheel_base, max_steer)
    BicycleModelRobot state{0.f, 0.f, 0.f, 3.0f, 1.0f, deg2rad(90.0f)};
    float k0 = 0.0f; // initial steer

    // initialize optimizer
    size_t max_iter = 100;
    float cost_th = 0.1f;
    TrajectoryParam delta_param{0.5f, 0.02f, 0.02f};
    float ds = 0.1f; // discretization of path length for trajectory generation
    TrajectoryOptimizer optim(state, max_iter, cost_th, delta_param, ds);


    // initialize table, ( x, y, yaw, x, km, kf)
    TableEntry first_entry{1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
    std::vector<TableEntry> table;
    table.push_back(first_entry);

    Arrow all_arr;

    float minx = 10.f, maxx = 30.f, dx = 5.f;
    float miny = 0.f, maxy = 20.f, dy = 2.f;
    float minyaw = deg2rad(-30.f), maxyaw = deg2rad(30.f), dyaw = deg2rad(30.f);
    for (float yaw = minyaw; yaw < maxyaw; yaw += dyaw) {
        for (float y = miny; y < maxy; y += dy) {
            for (float x = minx; x < maxx; x += dx) {
                Pose2D target_pose{x, y, yaw};

                TableEntry entry = findClosestEntry(table, x, y, yaw);
                
                // use closest entry as initial guess, p = (path len, middle_steer, final_steer)
                TrajectoryParam param{ std::hypot(x, y), entry.km, entry.kf};

                // get trajectory
                Pose2DTrajectory traj = optim.optimizeTrajectory(target_pose, param, k0);

                if (!traj.x.empty()) {
                    TableEntry new_entry{x, y, yaw, param.s, param.km, param.kf};
                    table.push_back(new_entry);

                    // generate plot
                    Arrow arr = trajectoryToVector(traj);
                    all_arr.insert(all_arr.end(), arr.begin(), arr.end());

                    // reflect over x-axis
                    for (auto& y : traj.y) y = -y;
                    Arrow reflect_arr = trajectoryToVector(traj);
                    all_arr.insert(all_arr.end(), reflect_arr.begin(), reflect_arr.end());
                }

            }
        }
    }

    std::cout << "Table size: " << table.size() << std::endl;

    // show plot
    Gnuplot gp;

    gp << "set term gif animate\n";
    gp << "set output '../animations/model_predictive_trajectory_generation.gif'\n";
    gp << "unset key \n";

    gp << "plot '-' with vectors \n";
    gp.send1d(all_arr);

    gp << "set output\n";


    // save to CSV
    std::ofstream myfile;
    myfile.open ("../misc/lookup_table.csv", std::ios::trunc); // overwrite old file
    myfile << "x,y,yaw,s,km,kf\n";
    for (auto& entry : table) {
        myfile << entry.x << ", " << entry.y << ", " << 
                  entry.yaw << ", " << entry.s << ", " <<
                  entry.km << ", " << entry.kf << "\n ";
    }
 
    myfile.close();


    return 0;
           

}
