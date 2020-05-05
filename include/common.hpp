#pragma once

#include <math.h>
#include <tuple>
#include <vector>


using Path = std::vector<std::pair<float, float>>;
using Arrow = std::vector<std::tuple<float, float, float, float>>;



float deg2rad(float deg);

// clip angle to [-pi, pi]
float normalizeAngle(float angle);

Arrow poseToVector(float x, float y, float theta, float r=0.5f);
