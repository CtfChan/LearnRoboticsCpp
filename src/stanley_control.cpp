#include "stanley_control.hpp"
#include "cubic_spline.hpp"


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
    
    float v = speed_gain_ * (target_vel_ - state.v);
    float detla = 0.f;


    return {v, delta};

}
