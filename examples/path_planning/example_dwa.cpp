#include "path_planning/dwa.hpp"

#include "gnuplot-iostream.h"


int main(int argc, char *argv[])
{
    Eigen::Vector2f goal;
    goal << 10.0, 10.0;

    //initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    RobotState x {0.0f, 0.0f, M_PI/8.0f, 0.0f, 0.0f}; // use {} allows aggregate intialization 

    Obstacles obs = {
        {-1, -1},
        {0, 2},
        {4.0, 2.0},
        {5.0, 4.0},
        {5.0, 5.0},
        {5.0, 6.0},
        {5.0, 9.0},
        {8.0, 9.0},
        {7.0, 9.0},
        {8.0, 10.0},
        {9.0, 11.0},
        {12.0, 13.0},
        {12.0, 12.0},
        {15.0, 15.0},
        {13.0, 13.0}
    };

    Config cfg;
    Trajectory traj;
    Gnuplot gp;

    gp << "set size ratio 1.0\n";
    gp << "set xrange [-2:16]\nset yrange [-2:20]\n";
    gp << "set term gif animate\n";
    gp << "set output '../animations/dwa.gif'\n";

    
    while (true) {
        auto [u, pred_trej] = dwaControl(x, cfg, goal, obs);
        x = motion(x, u, cfg.dt);
        traj.push_back(x);
        
        std::string plot_cmd = 
        "plot '-' title 'obstacles' pointtype 7,"
        "'-' title 'goal' pointsize 2 pointtype 2,"
        "'-' title 'robot' pointsize 2 pointtype 20,"
        "'-' title 'trajectory' pointsize 0.6 pointtype 26\n";
        gp << plot_cmd;
        gp.send1d(obs);
        gp.send1d(pointToVector(goal.x(), goal.y()));
        gp.send1d(pointToVector(x.x, x.y));
        gp.send1d(trajectorToVector(pred_trej));

        float dist_to_goal = std::hypot(x.x - goal.x(), x.y - goal.y() );
        if (dist_to_goal < cfg.robot_radius) {
            std::cout << "Arrived at goal! " << std::endl;
            break;
        }

    }

   std::string plot_cmd = 
        "plot '-' title 'obstacles' pointtype 7,"
        "'-' title 'goal' pointsize 2 pointtype 2,"
        "'-' title 'robot' pointsize 2 pointtype 20,"
        "'-' title 'final trajectory' pointsize 0.6 pointtype 26\n";    
    gp << plot_cmd;
    gp.send1d(obs);
    gp.send1d(pointToVector(goal.x(), goal.y()));
    gp.send1d(pointToVector(x.x, x.y));
    gp.send1d(trajectorToVector(traj));


    gp << "set output\n";


}