#pragma once

#include <vector>
#include <tuple>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <iostream>


struct Node {
    int x;
    int y;
    float cost;
    int parent;

    Node (int _x, int _y, float _cost, int _parent) : 
        x(_x), y(_y), cost(_cost), parent(_parent) {}

    Node () {}
};

class GridSearchBase {
public:
    GridSearchBase (std::vector<int>& ox,  std::vector<int>& oy, 
             float grid_res,  float robot_radius);

    virtual std::pair<std::vector<int>, std::vector<int>> 
        plan(float sx, float sy, float gx, float gy) = 0;

    std::pair<std::vector<int>, std::vector<int>> getExpandedNodes();

protected:
    void calculateObstacleMap( std::vector<int>& ox,  std::vector<int>& oy);

    std::pair<std::vector<int>, std::vector<int>> 
        calculateFinalPath(Node goal_node, std::unordered_map<int, Node>& closed_set) ;

    // converts position along X/Y axis into index
    int calculateXYIndex(int pos, int min_pos) ;

    // converts (x,y) into 1D index
    int calculateIndex(Node& node);

    // convert index position along X/Y axis into actual position
    float calculatePosition(int index, float minp);

    bool verifyNode(Node& node);

    std::vector<int> expanded_x_;
    std::vector<int> expanded_y_;

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




class Dijkstra : public GridSearchBase {
public:
    Dijkstra(std::vector<int>& ox,  std::vector<int>& oy, 
             float grid_res,  float robot_radius);

    virtual std::pair<std::vector<int>, std::vector<int>> 
        plan(float sx, float sy, float gx, float gy) override;

};




class Astar : public GridSearchBase {
public:
    Astar(std::vector<int>& ox,  std::vector<int>& oy, 
             float grid_res,  float robot_radius);

    virtual std::pair<std::vector<int>, std::vector<int>> 
        plan(float sx, float sy, float gx, float gy) override;

private:
    // euclidean distance as heuristic
    float calculateHeuristic(const Node& n1, const Node& n2);

};