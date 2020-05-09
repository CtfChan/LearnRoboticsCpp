#include "path_planning/grid_search.hpp"

#include "gnuplot-iostream.h"


int main() {
    std::vector<int> ox;
    std::vector<int> oy;
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
    gp << "set output '../animations/dijkstra.gif'\n";

    // all measurements in meters
    float sx = -5.0f;
    float sy = -5.0f;
    float gx = 50.0f;
    float gy = 50.0f;
    float grid_res = 2.0;
    float robot_radius = 1.0; 


    Dijkstra dijkstra = Dijkstra(ox, oy, grid_res, robot_radius);
    auto [rx, ry] = dijkstra.plan(sx, sy, gx, gy);

    
    // Do plotting
    auto [exp_x, exp_y] = dijkstra.getExpandedNodes();
    for (size_t i = 0; i < exp_x.size(); ++i) {
        std::vector<int> plot_exp_x(exp_x.begin(), exp_x.begin() + i);
        std::vector<int> plot_exp_y(exp_y.begin(), exp_y.begin() + i);

        gp << "plot '-' title 'obs', '-' title 'expanded'\n";
        gp.send1d(boost::make_tuple(ox, oy));
        gp.send1d(boost::make_tuple(plot_exp_x, plot_exp_y));
    }

    gp << "plot '-' title 'obs', '-' title 'path'\n";
    gp.send1d(boost::make_tuple(ox, oy));
    gp.send1d(boost::make_tuple(rx, ry));

    gp << "set output\n";


    return 0;
}