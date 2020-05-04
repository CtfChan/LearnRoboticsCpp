#pragma once

#include <math.h>
#include <tuple>
#include <vector>


float deg2rad(float deg);

// clip angle to [-pi, pi]
float normalizeAngle(float angle);

std::vector<std::tuple<float, float, float, float>> 
    poseToVector(float x, float y, float theta, float r=0.5f);
