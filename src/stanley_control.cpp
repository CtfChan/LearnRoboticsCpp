#include "stanley_control.hpp"


StanleyController::StanleyController(float control_gain, float speed_gain) :
    control_gain_(control_gain), speed_gain_(speed_gain) {
}

void StanleyController::setPath(const std::vector<float>& cx, 
    const std::vector<float>& cy, 
    const std::vector<float>& cyaw, 
    float target_vel) {
    cx_ = cx;
    cy_ = cy;
    cyaw_ = cyaw;
    target_vel_ = target_vel;
    has_arrived_ = false;
}

std::pair<float, float> StanleyController::computeControl(
    const BicycleModelRobot& state) {
    
    float a = speed_gain_ * (target_vel_ - state.v);
    float delta = 0.f;
    
    // find point on line closest to front axle pos (fx, fy)
    float fx = state.x + state.L * std::cos(state.yaw);
    float fy = state.y + state.L * std::sin(state.yaw);

    int idx = 0;
    float min_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < cx_.size(); ++i) {
        float dx = fx - cx_[i];
        float dy = fy - cy_[i];
        float d = std::hypot(dx, dy);
        if (d < min_dist) {
            min_dist = d;
            idx = i;
        }
    }

    if (idx == cx_.size()-1) {
        has_arrived_ = true;
        return {0.f, 0.f};
    }

    // cross track error
    // project error vector (dx, dy) onto
    // vector perpendicular to direction of travel and points towards line
    float front_axle_error = 
        -std::cos(state.yaw + M_PI/2.0f) * (fx - cx_[idx]) + 
        -std::sin(state.yaw + M_PI/2.0f) * (fy - cy_[idx]);

    // update tracking idx if greater equal to prev
    if (idx >= tracking_idx_) 
        tracking_idx_ = idx;

    // compute steer
    // for correcting heading error
    float theta_e = normalizeAngle(cyaw_[tracking_idx_] - state.yaw);
    // for correcting cross track error
    float theta_d = std::atan2(control_gain_ * front_axle_error, state.v);
    delta = theta_e + theta_d;

    return {a, delta};

}
