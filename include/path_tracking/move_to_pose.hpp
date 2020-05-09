# pragma once

#include <random>

#include "gnuplot-iostream.h"

#include "common.hpp"


struct Control {
    float v;
    float w;
};

// struct Pose {
//     float x;
//     float y;
//     float theta;
// };


class MoveToPoseController {
public:
    MoveToPoseController(float Kp_rho, float Kp_alpha, float Kp_beta) ;

    Control moveToPose(const Pose2D& curr) ;

    bool const hasArrived() const {
        return arrived_;
    }

    void setGoal(Pose2D& goal) {
        goal_ = goal;
    }

    
private:
    Pose2D goal_;

    bool arrived_ = false;
    float Kp_rho_;
    float Kp_alpha_;
    float Kp_beta_;

};

