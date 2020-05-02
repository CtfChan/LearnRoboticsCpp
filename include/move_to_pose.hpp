# pragma once

#include <random>

#include "gnuplot-iostream.h"

using Path = std::vector<std::pair<float, float>>;
using Arrow = std::vector<std::tuple<float, float, float, float>>;

struct Control {
    float v;
    float w;
};

struct Pose {
    float x;
    float y;
    float theta;
};

Arrow poseToArrow(const Pose& p, float arrow_len) {
    auto arr = std::make_tuple(
        p.x, 
        p.y, 
        arrow_len * std::cos(p.theta),
        arrow_len * std::sin(p.theta));
    return { arr };
}


class MoveToPoseController {
public:
    MoveToPoseController(float Kp_rho, float Kp_alpha, float Kp_beta) ;

    Control moveToPose(const Pose& curr) ;

    bool const hasArrived() const {
        return arrived_;
    }

    void setGoal(Pose& goal) {
        goal_ = goal;
    }

    
private:
    Pose goal_;

    bool arrived_ = false;
    float Kp_rho_;
    float Kp_alpha_;
    float Kp_beta_;

};

