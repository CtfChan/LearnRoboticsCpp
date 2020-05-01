
#include <vector>
#include <cmath>
#include <boost/tuple/tuple.hpp>

#include "gnuplot-iostream.h"




int main() {
	Gnuplot gp;

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


    gp << "set title 'Some Sample Plots'\n";
    gp << "set xlabel 'my xlabel'\n";
    gp << "set ylabel 'my ylabel'\n";
	gp << "set xrange [-20:70]\nset yrange [-20:70]\n";

    gp << "plot '-'\n";
    gp.send1d(boost::make_tuple(ox, oy));


    // animation robot moves upwards
    std::vector<int> posx = {0};
    std::vector<int> poxy = {0};
    for (int i = 0; i < 20; ++i) {
        // plot obs and robot
        gp << "plot '-' title 'obs', '-' title 'robot'\n";
        gp.send1d(boost::make_tuple(ox, oy));

        // plot robot
        poxy[0] = i;
        gp.send1d(boost::make_tuple(posx, poxy));

        sleep(1.0);

    }

}