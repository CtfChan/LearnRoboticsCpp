# pragma once

#include <iostream>
#include <vector>
#include <random>

#include "gnuplot-iostream.h"


struct Node {
    float x;
    float y; 
    std::vector<float> path_x;
    std::vector<float> path_y;
    int parent = -1;

    Node (float _x, float _y) : x(_x), y (_y) {}

    Node () {}
};

using CircleObstacle = std::tuple<float, float, float>;


class RRT {
public:
    RRT(float min_rand_, float max_rand_, std::vector<CircleObstacle>& obs, 
        float expand_dis=3.0f, float path_res=0.25, int goal_sample_rate=5,
        size_t max_iter=500);

    std::pair<std::vector<float>, std::vector<float>> 
        plan(float sx, float sy, float gx, float gy, Gnuplot& gp);

private:

    Node generateRandomNode();

    size_t nearestNodeIndex(std::vector<Node>& nodes, Node& query);

    Node steer(Node& from_node, Node& to_node, float extend_length);

    bool noCollision(Node& n);

    std::pair<std::vector<float>, std::vector<float>>
        generateFinalCourse(std::vector<Node>& node_list);

    std::pair<float, float> calculateDistanceAndAngle(Node& s, Node& g);

    float calculateDistanceToGoal(Node& n);


    std::vector<CircleObstacle> obs_;  
    float min_rand_;
    float max_rand_;
    float expand_dis_;
    float path_res_;
    float goal_sample_rate_;
    size_t max_iter_;

    Node goal_node_;
    Node start_node_;
    
};