#include "path_planning/rrt.hpp"
#include "gnuplot-iostream.h"



int main(int argc, char *argv[])
{
    Gnuplot gp;

    std::vector<CircleObstacle> obs = {
        {5, 5, 1},
        {3, 6, 2},
        {3, 8, 2},
        {3, 10, 2},
        {7, 5, 2},
        {9, 5, 2},
        {8, 10, 1}
    };

    gp << "set size ratio 1.0\n";
    gp << "set xrange [-2:15]\nset yrange [-2:18]\n";
    gp << "set term gif animate\n";
    gp << "set output '../animations/rrt.gif'\n";


    float sx = 0.0;
    float sy = 0.0;
    float gx = 6.0;
    float gy = 10.0;
    
    float min_rand = -2.0;
    float max_rand = 15.0;

    RRT rrt(min_rand, max_rand, obs);
    auto [rx, ry] = rrt.plan(sx, sy, gx, gy, gp);


    gp << "set output\n";



    return 0;
}