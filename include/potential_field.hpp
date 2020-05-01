# pragma once

#include "gnuplot-iostream.h"

using Path = std::vector<std::pair<float, float>> ;


class PotentialField {
public:
    PotentialField(const std::vector<float>& ox, const std::vector<float>& oy,
                float reso, float robot_radius, 
                float kp=5.0f, float eta=100.0f, float area_width=30.0f);

    Path plan(float sx, float sy, float gx, float gy, Gnuplot& gp);

private:
    void generatePotentialMap(float gx, float gy);

    float calculateAttractivePotential(float x, float y, float gx, float gy);

    float calculateRepulsivePotential(float x, float y);

    float minx_, maxx_, miny_, maxy_; // bounds of pmap
    std::vector<std::vector<float>> pmap_;
    std::vector<std::tuple<float, float, float>> viz_pmap_; // for visualizing gradients


    std::vector<float> ox_;
    std::vector<float> oy_;
    float reso_;
    float rr_; // robot radius
    float kp_;
    float eta_;
    float area_width_; // padding for map

    std::vector<std::pair<int, int>> motion_model_ = {
        {1, 0},
        {0, 1},
        {-1, 0},
        {0, -1},
        {-1, -1},
        {-1, 1},
        {1, -1},
        {1, 1}
    };

};