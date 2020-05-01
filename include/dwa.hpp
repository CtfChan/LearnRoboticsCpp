# pragma once



#include "gnuplot-iostream.h"
#include <Eigen/Core>



enum class RobotType {
    circle,
    rectangle
};

struct Config {
    float max_speed = 1.0f; // m/s
    float min_speed = -0.5f;
    float max_yawrate = 40.0f * M_PI / 180.0f; // rad/s
    float max_accel = 0.2f;
    float max_dyawrate = 40.0f * M_PI / 180.0f;
    float v_reso = 0.01;
    float yawrate_reso = 0.1f * M_PI / 180.0f;
    float dt = 0.1f; // s
    float predict_time = 3.0;
    float to_goal_cost_gain = 0.15;
    float speed_cost_gain = 1.0;
    float obstacle_cost_gain = 1.0;

    // ROBOT TYPE
    RobotType robot_type = RobotType::circle;

    // for circular robot
    float robot_radius = 1.0; // m

    // for rectangular robot
    float robot_width = 0.5;
    float robot_length = 1.2;

};


struct RobotState {
    float x;
    float y;
    float theta;
    float v;
    float w;
};

struct Control {
    float v;
    float w;
};

struct DynamicWindow {
    float vmin;
    float vmax;
    float yaw_rate_min;
    float yaw_rate_max;
};

using Trajectory = std::vector<RobotState>;
using Obstacles = std::vector<std::pair<float, float>>;

// covenient functions for plotting


std::vector<std::pair<float, float>> pointToVector(float x, float y) {
    return { {x, y}};
}


std::vector<std::pair<float, float>> trajectorToVector(const Trajectory& traj) {
    std::vector<std::pair<float, float>> vec;
    vec.reserve(traj.size());
    for (const auto& pose : traj) 
        vec.emplace_back(pose.x, pose.y);
    return vec;
}


// DWA  functions

RobotState motion(const RobotState& x, const Control& u, float dt);

std::pair<Control, Trajectory> dwaControl(const RobotState& x, 
    const Config& cfg, const Eigen::Vector2f& goal, const Obstacles& obs);

DynamicWindow calculateDynamicWindow(const RobotState& x, const Config& cfg);

Trajectory predictTrajectory(const RobotState& x_init, 
    float v, float w, const Config& cfg);


std::pair<Control, Trajectory> calculateControlAndTrajectory(
    const RobotState& x, const DynamicWindow& dw, 
    const Config& cfg, const Eigen::Vector2f& goal, const Obstacles& obs);


float calculateObstacleCost(const Trajectory& traj, const Obstacles& obs, 
    const Config& cfg);

float calculateToGoalCost(const Trajectory& traj, const Eigen::Vector2f& goal);
