#pragma once

#include "common.hpp"

#include <algorithm>



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