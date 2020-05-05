

#include "move_to_pose.hpp"

MoveToPoseController::MoveToPoseController(float Kp_rho, float Kp_alpha, 
    float Kp_beta) :  Kp_rho_(Kp_alpha), 
    Kp_alpha_(Kp_alpha), Kp_beta_(Kp_beta) {}

Control MoveToPoseController::moveToPose(const Pose& curr)  {
    Control u {0.f, 0.f};

    float dx = goal_.x - curr.x;
    float dy = goal_.y - curr.y;

    // clip angles to [-pi, pi]
    float rho = std::hypot(dx, dy);
    float alpha = fmod(std::atan2(dy, dx) - curr.theta + M_PI, 2 * M_PI) - M_PI;
    float beta  = fmod(goal_.theta - curr.theta - alpha + M_PI, 2 * M_PI) - M_PI;

    float angle_diff = fmod(goal_.theta - curr.theta + M_PI, 2 * M_PI) - M_PI;

    if (rho < 0.1 && std::abs(angle_diff) < 5.0f * M_PI/180.f ) {
        arrived_ = true;
        return u;
    }

    u.v = Kp_rho_ * rho;
    u.w = Kp_alpha_ * alpha + Kp_beta_ * beta;

    if (alpha > M_PI/2.f || alpha < -M_PI/2.f )
        u.v = -u.v;

    return u;
}



