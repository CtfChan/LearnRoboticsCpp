#include "gnuplot-iostream.h"
#include "path_planning/cubic_spline.hpp"

int main() {
    std::vector<float> x = {-2.5f, 0.0f, 2.5f, 5.0f, 7.5f, 3.0f, -1.0f};
    std::vector<float> y = {0.7f, -6.0f, 5.0f, 6.5f, 0.0f, 5.0f, -2.0f};
    float ds = 0.1f; // [m], distance between each interpolated points
    
    auto [rx, ry, ryaw, rk, rs] = calculateSplineCourse(x, y, ds);

    Gnuplot gp;
    gp << "set term gif animate\n";
    gp << "set output '../animations/cubic_spline.gif'\n";

    gp << "set size 1,1 \n";
    gp << "set multiplot\n";
    gp << "unset key\n";

    // plot spline
    gp << "set size 0.5, 0.5\n";
    gp << "set origin 0.0, 0.5\n"; // origin is lower left corner of graph
    gp << "set title 'Spline'\n";
    gp << "plot '-' with line title 'spline',"
          "'-' title 'input'\n";
    gp.send1d(boost::make_tuple(rx, ry));
    gp.send1d(boost::make_tuple(x, y));

    // plot yaw and line len
    gp << "set size 0.5, 0.5\n";
    gp << "set origin 0.5, 0.5\n"; 
    gp << "set title 'Yaw .vs Arc Length'\n";
    for (auto& yaw : ryaw) 
        yaw = yaw * 180.f/M_PI;
    gp << "plot '-' with line \n";
    gp.send1d(boost::make_tuple(rs, ryaw));


    gp << "set size 0.5, 0.5\n";
    gp << "set origin 0.0, 0.0\n"; 
    gp << "set title 'Curvature vs. Arc Length'\n";

    gp << "plot '-' with line \n ";
    gp.send1d(boost::make_tuple(rs, rk));

    gp << "unset multiplot\n";
    gp << "set output\n";

    return 0;
}