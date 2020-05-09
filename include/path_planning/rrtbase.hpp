#pragma once


struct Node {
    float x;
    float y; 
    std::vector<float> path_x;
    std::vector<float> path_y;
    int parent = -1;
    int idx = -1; // position in nodes_list_
    float cost = 0.0f;

    Node (float _x, float _y) : x(_x), y (_y) {}

    Node () {}
};


using CircleObstacle = std::tuple<float, float, float>;


class RRTBase {
public:
    virtual std::pair<std::vector<float>, std::vector<float>> 
        plan(float sx, float sy, float gx, float gy) = 0;
    
protected:
    Node generateRandomNode();

    size_t nearestNodeIndex(Node& query);

    // returns new node who's parent is from_node
    Node steer(Node& from_node, Node& to_node, float extend_length=std::numeric_limits<float>::infinity());

    bool noCollision(Node& n);

    std::pair<std::vector<float>, std::vector<float>>
        generateFinalCourse(int goal_ind);

    std::pair<float, float> calculateDistanceAndAngle(Node& s, Node& g);

    float calculateDistanceToGoal(Node& n);

    int searchBestGoalNode();


    std::vector<Node> nodes_list_;

    std::vector<CircleObstacle> obs_;  
    float min_rand_;
    float max_rand_;
    float expand_dis_;
    float path_res_;
    float goal_sample_rate_;
    size_t max_iter_;
    // float connect_circle_dist_;

    Node goal_node_;
    Node start_node_;
    


};

class RRT : public RRTBase {
public:



};
