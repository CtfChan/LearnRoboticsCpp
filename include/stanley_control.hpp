#pragma once

#include "common.hpp"

#include <algorithm>

struct BicycleModelRobot {
    float x;
    float y;
    float yaw;
    float v;
    float L; // wheelbase
    float max_steer;

    // acceleration, steering, dt
    void update(float acc, float delta, float dt) {
        delta = std::clamp(delta, -max_steer, max_steer);
        
        x += v * std::cos(yaw) * dt;
        y += v * std::sin(yaw) * dt;
        yaw += v / L * std::tan(delta) * dt;
        yaw = normalizeAngle(yaw);
        v += acc * dt;
    }
};


class StanleyController {
public:
    StanleyController(float control_gain, float speed_gain);

    void setPath(const std::vector<float>& cx, 
        const std::vector<float>& cy, 
        const std::vector<float>& cyaw, 
        float target_vel);

    // acc and steer
    std::pair<float, float> computeControl(const BicycleModelRobot& state);

    bool hasArrived() const {
        return has_arrived_;
    }

private:

    // params
    float control_gain_;
    float speed_gain_;

    // trajectory to follow
    std::vector<float> cx_, cy_, cyaw_;
    int tracking_idx_ = -1;
    float target_vel_;

    bool has_arrived_ = true;


};