#include "gnuplot-iostream.h"
#include "path_planning/prm.hpp"





int main(int argc, char *argv[])
{
    std::vector<float> ox;
    std::vector<float> oy;
    for (int i = -10; i < 60; ++i) {
        ox.push_back(i);
        oy.push_back(-10);
    }
    for (int i = -10; i < 61; ++i) {
        ox.push_back(60);
        oy.push_back(i);
    }
    for (int i = -10; i < 61; ++i) {
        ox.push_back(i);
        oy.push_back(60);
    }
    for (int i = -10; i < 61; ++i) {
        ox.push_back(-10);
        oy.push_back(i);
    }
    for (int i = -10; i < 40; ++i) {
        ox.push_back(20);
        oy.push_back(i);
    }
    for (int i = 0; i < 40; ++i) {
        ox.push_back(40);
        oy.push_back(60 - i);
    }

    Gnuplot gp;

    gp << "set size ratio 1.0\n";
    gp << "set xrange [-20:70]\nset yrange [-20:80]\n";
    gp << "set term gif animate\n";
    gp << "set output '../animations/prm.gif'\n";

    // all measurements in meters
    float sx = -5.0f;
    float sy = -5.0f;
    float gx = 50.0f;
    float gy = 50.0f;
    float robot_radius = 5.0f; 

    ProbabilisticRoadmap prm(ox, oy, robot_radius, 500, 10, 30.0f);
    auto [rx, ry] = prm.plan(sx, sy, gx, gy);
    auto plot_map = prm.getRoadMap();
    auto [exp_x, exp_y] = prm.getExpandedNodes();

    // show dijkstra expansion
    for (size_t i = 0; i < exp_x.size(); ++i) {
        std::vector<float> plot_exp_x(exp_x.begin(), exp_x.begin() + i);
        std::vector<float> plot_exp_y(exp_y.begin(), exp_y.begin() + i);

        gp << "plot '-' with vectors nohead title 'roadmap',"
                   "'-' title 'obs'," 
                   "'-' title 'expanded'\n";
        gp.send1d(plot_map);
        gp.send1d(boost::make_tuple(ox, oy));
        gp.send1d(boost::make_tuple(plot_exp_x, plot_exp_y));

    }

    // final plot with path
    gp << "plot '-' with vectors nohead title 'roadmap', '-' title 'obs', '-' with linespoints lw 5 title 'path'\n";
	gp.send1d(plot_map);
    gp.send1d(boost::make_tuple(ox, oy));
    gp.send1d(boost::make_tuple(rx, ry));

    gp << "set output\n";
    
    return 0;
}