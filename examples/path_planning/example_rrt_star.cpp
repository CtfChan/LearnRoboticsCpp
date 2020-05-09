#include "path_planning/rrt_star.hpp"


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
        {8, 10, 1},
        {6, 12, 1}
    };

    gp << "set size ratio 1.0\n";
    gp << "set xrange [-2:15]\nset yrange [-2:18]\n";
    gp << "set term gif animate\n";
    gp << "set output '../animations/rrt_star.gif'\n";

    float sx = 0.0;
    float sy = 0.0;
    float gx = 6.0;
    float gy = 10.0;
    
    float min_rand = -2.0;
    float max_rand = 15.0;

    RRTStar rrt_star(min_rand, max_rand, obs);
    auto [rx, ry] = rrt_star.plan(sx, sy, gx, gy, gp);


    gp << "set output\n";



    return 0;
}