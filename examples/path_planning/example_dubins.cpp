#include "common.hpp"
#include "gnuplot-iostream.h"

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <boost/program_options.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

int main() {
    ob::StateSpacePtr space(std::make_shared<ob::DubinsStateSpace>());
    ob::ScopedState<> from(space), to(space), s(space);

    from[0] = 0.0;
    from[1] = 0.0;
    from[2] = 0.0;

    to[0] = 1.0;
    to[1] = 3.0;
    to[2] = 1.0;

    Gnuplot gp;

    gp << "set term png\n";
    gp << "set output '../animations/dubins.png'\n";

    // plot start and goal
    gp << "plot '-' with vector title 'start' lw 2,"
          "'-' with vector title 'goal' lw 2,"
          "'-' with vector title 'trajectory' lw 2\n";

    gp.send1d(poseToVector(from[0], from[1], from[2], 0.5));
    gp.send1d(poseToVector(to[0], to[1], to[2], 0.5));

    // plot trajectory
    std::vector<std::tuple<float, float, float, float>> traj;
    const unsigned int num_pts = 50;
    std::vector<double> reals;

    std::cout << "distance: " << space->distance(from(), to()) << "\npath:\n";
    for (unsigned int i = 0; i <= num_pts; ++i) {
        space->interpolate(from(), to(), (double)i / num_pts, s());
        reals = s.reals();
        float x = static_cast<float>(reals[0]);
        float y = static_cast<float>(reals[1]);
        float theta = static_cast<float>(reals[2]);
        auto arrow = poseToVector(x, y, theta, 0.2f);
        traj.push_back(arrow[0]);
    }
    gp.send1d(traj);

    gp << "set output\n";
}