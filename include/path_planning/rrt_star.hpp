#pragma once

#include <iostream>
#include <vector>
#include <random>
#include <math.h>

#include "gnuplot-iostream.h"

struct Node
{
    float x;
    float y;
    std::vector<float> path_x;
    std::vector<float> path_y;
    int parent = -1;
    int idx = -1; // position in nodes_list_
    float cost = 0.0f;

    Node(float _x, float _y) : x(_x), y(_y) {}

    Node() {}
};

using CircleObstacle = std::tuple<float, float, float>;

class RRTStar
{
public:
    RRTStar(float min_rand_, float max_rand_, std::vector<CircleObstacle> &obs,
            float expand_dis = 3.0f, float path_res = 0.1f, int goal_sample_rate = 5,
            size_t max_iter = 500, float connect_circle_dist = 50.0f);

    std::pair<std::vector<float>, std::vector<float>>
    plan(float sx, float sy, float gx, float gy, Gnuplot &gp);

private:
    Node generateRandomNode();

    size_t nearestNodeIndex(Node &query);

    // returns new node who's parent is from_node
    Node steer(Node &from_node, Node &to_node, float extend_length = std::numeric_limits<float>::infinity());

    bool noCollision(Node &n);

    std::pair<std::vector<float>, std::vector<float>>
    generateFinalCourse(int goal_ind);

    std::pair<float, float> calculateDistanceAndAngle(Node &s, Node &g);

    float calculateDistanceToGoal(Node &n);

    int searchBestGoalNode();

    // returns indices of nearby nodes
    std::vector<size_t> findNearNodes(Node &n);

    // returns idx of best parent, -1 if no valid parent
    // will modify n's cost and parent by reference
    int chooseParent(Node &n, std::vector<size_t> &near_nodes);

    float calculateNewCost(Node &from_node, Node &to_node);

    void rewire(Node &n, std::vector<size_t> &near_nodes);

    void propagateCostToLeaves(Node &parent_node);

    std::vector<Node> nodes_list_;

    std::vector<CircleObstacle> obs_;
    float min_rand_;
    float max_rand_;
    float expand_dis_;
    float path_res_;
    float goal_sample_rate_;
    size_t max_iter_;
    float connect_circle_dist_;

    Node goal_node_;
    Node start_node_;
};