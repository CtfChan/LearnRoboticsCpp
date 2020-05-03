# pragma once

#include <math.h>
#include <tuple>

float deg2rad(float deg) {
    return deg * M_PI/ 180.0f;
}

std::vector<std::tuple<float, float, float, float>> 
    poseToVector(float x, float y, float theta, float r=0.5f) {
    float dx = r * std::cos(theta);
    float dy = r * std::sin(theta);
    auto arrow = std::make_tuple(x, y, dx ,dy);
    return { arrow  };
}