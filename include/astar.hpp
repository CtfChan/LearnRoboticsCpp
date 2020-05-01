# pragma once

#include <vector>
#include <tuple>
#include <unordered_set>
#include <unordered_map>
#include <cmath>

#include "gnuplot-iostream.h"


struct Node {
    int x;
    int y;
    float cost;
    int parent;

    Node (int _x, int _y, float _cost, int _parent) : 
        x(_x), y(_y), cost(_cost), parent(_parent) {}

    Node () {}
};

class AStar {
    
public:
    AStar( std::vector<int>& ox,  std::vector<int>& oy, 
             float grid_res,  float robot_radius);

    std::pair<std::vector<int>, std::vector<int>> plan(float sx, float sy, float gx, float gy, Gnuplot& gp);


private:
    // euclidean distance as heuristic
    float calculateHeuristic(const Node& n1, const Node& n2) {
        return std::hypot(n1.x - n2.x, n1.y - n2.y);
    }

    void calculateObstacleMap( std::vector<int>& ox,  std::vector<int>& oy);

    std::pair<std::vector<int>, std::vector<int>> calculateFinalPath(Node goal_node, 
                                            std::unordered_map<int, Node>& closed_set);

    int calculateXYIndex(int pos, int min_pos) {
        return std::round((pos - min_pos) / grid_res_);
    }


    int calculateIndex(Node& node) {
        return (node.y - min_y_) * x_width_ + (node.x - min_x_);
    }

    float calculatePosition(int index, float minp) {
        float pos = index * grid_res_ + minp;
        return pos;
    }

    bool verifyNode(Node& node) {
        float px = calculatePosition(node.x, min_x_);
        float py = calculatePosition(node.y, min_y_);

        if (px < min_x_)
            return false;
        if (py < min_y_)
            return false;
        if (px >= max_x_)
            return false;
        if (py >= max_y_)
            return false;

        if (obstacle_map_[node.x][node.y])
            return false;

        return true;
    }


    float min_x_;
    float max_x_;
    int x_width_; // grid cells along x

    float min_y_;
    float max_y_;
    int y_width_; // grid cells along y

    float grid_res_;
    float robot_radius_;

    std::vector<std::vector<bool>> obstacle_map_;
    std::vector<int> ox_;
    std::vector<int> oy_;

    std::vector<std::tuple<int, int, float> > motion_model_ = {
        std::make_tuple(1, 0, 1),
        std::make_tuple(0, 1, 1),
        std::make_tuple(-1, 0, 1),
        std::make_tuple(0, -1, 1),
        std::make_tuple(-1, -1, std::sqrt(2)),
        std::make_tuple(-1, 1, std::sqrt(2)),
        std::make_tuple(1, -1, std::sqrt(2)),
        std::make_tuple(1, 1, std::sqrt(2))
    };
    

};

