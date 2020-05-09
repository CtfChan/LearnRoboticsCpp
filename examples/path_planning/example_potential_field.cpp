
#include "common.hpp"
#include "path_planning/potential_field.hpp"


#include "gnuplot-iostream.h"


int main(int argc, char *argv[])
{
    float sx = 0.0f;
    float sy = 10.0f;
    float gx = 30.0f;
    float gy = 30.0f;

    float grid_size = 0.5f;
    float robot_radius = 5.0f;

    std::vector<float> ox = {15.0f, 5.0f, 20.0f, 25.0f};
    std::vector<float> oy = {25.0f, 15.0f, 26.0f, 25.0f};

    Gnuplot gp;
    
    gp << "set size ratio 1.0\n";
    gp << "set xrange [0:40]\nset yrange [0:40]\n";
    gp << "set term gif animate\n";
    gp << "set output '../animations/potential_field.gif'\n";

    PotentialField pf(ox, oy, grid_size, robot_radius);
    Path path = pf.plan(sx, sy, gx, gy);


    for (size_t i = 0; i < path.size(); ++i) {
        Path plot_path(path.begin(), path.begin()+i);
        gp << "plot '-' title 'path',"
            "'-' title 'goal',"
            "'-' title 'obs' pointsize 4 pointtype 7\n";
        gp.send1d(plot_path);
        std::vector<std::tuple<float, float>> goal_plot = { {gx, gy}};
        gp.send1d(goal_plot);
        gp.send1d(boost::make_tuple(ox, oy));
    }

    gp << "set output\n";


    return 0;
}