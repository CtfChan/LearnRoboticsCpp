# include "rrt_star.hpp"


RRTStar::RRTStar(float min_rand, float max_rand, std::vector<CircleObstacle>& obs, 
        float expand_dis, float path_res, int goal_sample_rate,
        size_t max_iter, float connect_circle_dist) : 
        min_rand_(min_rand), max_rand_(max_rand), obs_(obs),
        expand_dis_(expand_dis), path_res_(path_res), 
        goal_sample_rate_(goal_sample_rate), max_iter_(max_iter),
        connect_circle_dist_(connect_circle_dist) {
    // initialize params
}



 std::pair<std::vector<float>, std::vector<float>> 
    RRTStar::plan(float sx, float sy, float gx, float gy, Gnuplot& gp){
    
    start_node_ = Node(sx, sy);
    goal_node_ = Node(gx, gy);

    nodes_list_.clear();
    nodes_list_.push_back(start_node_);
    nodes_list_[0].idx = 0;

    for (size_t i = 0; i < max_iter_; ++i) {
        std::cout << "Iteration #: " << i << std::endl;

        // sample new new node and find nearest neighbour
        Node random_node = generateRandomNode();
        size_t nearest_ind = nearestNodeIndex(random_node);
        Node nearest_node = nodes_list_[nearest_ind];

        Node new_node = steer(nearest_node, random_node, expand_dis_);
        new_node.idx = nodes_list_.size();

        if (noCollision(new_node)) {
            std::vector<size_t> near_inds = findNearNodes(new_node);
            int new_parent_ind = chooseParent(new_node, near_inds);
            if (new_parent_ind != -1) {
                nodes_list_.push_back(new_node);
                rewire(new_node, near_inds);
            }
        }

        // plot
        gp << "plot '-' with circles title 'obstacles' fill solid, '-'  with vectors nohead title 'tree', '-' title 'next' pointtype 26\n";
        gp.send1d(obs_);
    
        std::vector<std::tuple<float, float, float, float>> disp_nodes;
        for (auto& node : nodes_list_) {
            if (node.parent != -1) {
                int p_id = node.parent;
                float dx = nodes_list_[p_id].x - node.x;
                float dy = nodes_list_[p_id].y - node.y;
                disp_nodes.emplace_back(node.x, node.y, dx, dy);
            }
        }
        gp.send1d(disp_nodes);

        std::vector<std::pair<float, float>> rand_plot = { {random_node.x, random_node.y} };
        gp.send1d(rand_plot);
        // sleep(1);
    }

    // look for best goal node, return path
    int last_ind = searchBestGoalNode();
    if (last_ind != -1) {
        auto [rx, ry] = generateFinalCourse(last_ind);

        gp << "plot '-' with circles title 'obstacles' fill solid, '-'  with vectors nohead title 'tree', '-' with linespoints lw 5 title 'path'\n";
        
        std::vector<std::tuple<float, float, float, float>> disp_nodes;
        for (auto& node : nodes_list_) {
            if (node.parent != -1) {
                int p_id = node.parent;
                float dx = nodes_list_[p_id].x - node.x;
                float dy = nodes_list_[p_id].y - node.y;
                disp_nodes.emplace_back(node.x, node.y, dx, dy);
            }
        }
        
        gp.send1d(obs_);
        gp.send1d(disp_nodes);
        gp.send1d(boost::make_tuple(rx, ry));


        return {rx, ry};
    }


    return {{}, {}};


}

int RRTStar::searchBestGoalNode() {
    int best_ind = -1;
    float best_cost = std::numeric_limits<float>::infinity();

    // look for nodes sufficiently close to goal
    for (size_t i = 0; i < nodes_list_.size(); ++i) {
        Node& curr_node = nodes_list_[i];
        if (calculateDistanceToGoal(curr_node) < expand_dis_) {
            
            Node n = steer(curr_node, goal_node_);
            if (noCollision(n) && n.cost < best_cost ) {
                best_ind = i;
                best_cost = n.cost;
            }

        }
    }

    return best_ind;
}



std::vector<size_t> RRTStar::findNearNodes(Node& n) {
    int nnodes = nodes_list_.size() + 1; // avoid div by 0
    float r = connect_circle_dist_ * std::sqrt( std::log(nnodes) / nnodes  );
    r = std::min(r, expand_dis_);

    std::vector<size_t> near_inds;
    for (size_t i = 0; i < nodes_list_.size(); ++i) {
      
        auto [d, theta] = calculateDistanceAndAngle(n, nodes_list_[i]);
        if (d <= r) 
            near_inds.push_back(i);
    }

    return near_inds;
}




int RRTStar::chooseParent(Node& n, std::vector<size_t>& near_nodes) {
    if (near_nodes.empty())
        return -1;

    // find cheapest available parent by cost
    float min_cost = std::numeric_limits<float>::max();
    int min_ind = -1;
    for (size_t i = 0; i < near_nodes.size(); ++i) {
        Node& near_node = nodes_list_[near_nodes[i]];
        Node t_node = steer(near_node, n);
        if (noCollision(t_node) && calculateNewCost(near_node, n) < min_cost) {
            min_ind = i;
            min_cost = calculateNewCost(near_node, n);
        }
    }
    
    // none found
    if (min_ind == -1)
        return min_ind;

    // create a replacement for n with new parent
    Node& closest_parent = nodes_list_[near_nodes[min_ind]];
    Node new_node = steer(closest_parent, n);
    new_node.idx = n.idx; 
    new_node.parent = closest_parent.idx;
    new_node.cost = calculateNewCost(closest_parent, n);
    n = new_node;


    // std::cout << "better parent found: " << closest_parent.idx << std::endl;
    // std::cout << "new_node parent: " << new_node.parent << std::endl;

    return near_nodes[min_ind];
}

float RRTStar::calculateNewCost(Node& from_node, Node& to_node) {
    auto [d, theta] = calculateDistanceAndAngle(from_node, to_node);
    return from_node.cost + d;
}


void RRTStar::rewire(Node& n, std::vector<size_t>& near_nodes) {
    for (size_t near_node_idx : near_nodes) {
        Node near_node = nodes_list_[near_node_idx]; // original node
        Node edge_node = steer(n, near_node); // potential new node, with cheaper edge cost

        edge_node.cost = calculateNewCost(n, near_node);
        edge_node.idx = near_node.idx;

        if (noCollision(edge_node) && edge_node.cost < near_node.cost) {
            
            nodes_list_[near_node_idx] = edge_node;
            propagateCostToLeaves(n);
        }

    }
}


void RRTStar::propagateCostToLeaves(Node& parent_node) {

    for (Node& node: nodes_list_) {
        if (node.parent == parent_node.idx) {

            node.cost = calculateNewCost(parent_node, node);
            propagateCostToLeaves(node);
        }
    }
}


std::pair<std::vector<float>, std::vector<float>> RRTStar::generateFinalCourse(int goal_ind) {
    std::vector<float> rx, ry;
    Node& node = nodes_list_[goal_ind];
    while ( node.parent != -1) {
        rx.push_back(node.x);
        ry.push_back(node.y);
        node = nodes_list_[node.parent];
    }

    return {rx, ry};
    
}



bool RRTStar::noCollision(Node& n) {

    // for every obstacle, compare with points in the node's path
    for (auto [ox, oy, r] : obs_) {
        for (size_t i = 0; i < n.path_x.size(); ++i) {
            float dx = ox - n.path_x[i];
            float dy = oy - n.path_y[i];
            float d = std::hypot(dx, dy);

            // robot is inside obs
            if (d <= r) 
                return false;
        }
    }

    return true;
}



Node RRTStar::steer(Node& from_node, Node& to_node, float extend_length) {
    Node new_node(from_node.x, from_node.y);
    new_node.path_x.push_back(new_node.x);
    new_node.path_y.push_back(new_node.y);

    auto [d, theta] = calculateDistanceAndAngle(new_node, to_node);

    // cap the extension length
    extend_length = std::min(d, extend_length);
    
    // generate trajectory to to_node
    int n_expand = std::floor(extend_length / path_res_);
 
    for (int i = 0; i < n_expand; ++i) {
        new_node.x += path_res_ * std::cos(theta);
        new_node.y += path_res_ * std::sin(theta);
        new_node.path_x.push_back(new_node.x);
        new_node.path_y.push_back(new_node.y);
    }

    // add last node if necessary
    auto [final_d, final_theta] = calculateDistanceAndAngle(new_node, to_node);
    if (final_d <= path_res_) {
        new_node.path_x.push_back(to_node.x);
        new_node.path_y.push_back(to_node.y);
    }

    new_node.parent = from_node.idx;

    return new_node;

}



Node RRTStar::generateRandomNode() {
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> goal_dis(0, 100); // determines if we need to sample goal
    std::uniform_real_distribution<> point_dis(min_rand_, max_rand_); 

    Node n;
    if (goal_dis(gen) > goal_sample_rate_) {
        n.x = point_dis(gen);
        n.y = point_dis(gen);
    } else {
        n.x = goal_node_.x;
        n.y = goal_node_.y;
    }

    return n;
}


size_t RRTStar::nearestNodeIndex(Node& query) {
    auto it = std::min_element(nodes_list_.begin(), nodes_list_.end(), 
    [&](Node& n1, Node& n2) {
        float dx1 = n1.x - query.x;
        float dy1 = n1.y - query.y;

        float dx2 = n2.x - query.x;
        float dy2 = n2.y - query.y;

        return std::hypot(dx1, dy1) < std::hypot(dx2, dy2);
    } );

    return std::distance(nodes_list_.begin(), it);
}

// from s to g
std::pair<float, float> RRTStar::calculateDistanceAndAngle(Node& s, Node& g) {
    float dx = g.x - s.x;
    float dy = g.y - s.y;
    float d = std::hypot(dx, dy);
    float theta = std::atan2(dy, dx);
    return {d, theta};
}

float RRTStar::calculateDistanceToGoal(Node& n) {
    float dx = n.x - goal_node_.x;
    float dy = n.y - goal_node_.y;
    return std::hypot(dx, dy);
}



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