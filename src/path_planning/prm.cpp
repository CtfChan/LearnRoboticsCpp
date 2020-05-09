#include "path_planning/prm.hpp"


std::pair<std::vector<float>, std::vector<float>> ProbabilisticRoadmap::getExpandedNodes() {
    return {expanded_x_, expanded_y_};
}

std::vector<boost::tuple<float, float, float, float>> ProbabilisticRoadmap::getRoadMap() {
    return plot_map_;
}

ProbabilisticRoadmap::ProbabilisticRoadmap(std::vector<float>& ox, std::vector<float>& oy, 
        float robot_radius, 
        int sample_points, int n_knn, float max_edge_len) :
        ox_(ox), oy_(oy), robot_radius_(robot_radius), kdtree_(ox, oy),
        sample_points_(sample_points), n_knn_(n_knn), max_edge_len_(max_edge_len) {

}


std::pair<std::vector<float>, std::vector<float>> 
        ProbabilisticRoadmap::plan(float sx, float sy, float gx, float gy){
    auto [sample_x, sample_y] = samplePoints(sx, sy, gx, gy);

    std::vector<std::vector<int>> road_map = generateRoadMap(sample_x, sample_y);

    // for plotting road map, save as (x, y, dx, dy) tuple
    for (size_t i = 0; i < road_map.size(); ++i) {
        float x = sample_x[i];
        float y = sample_y[i];
        for (size_t j = 0; j < road_map[i].size(); ++j) {
            int idx = road_map[i][j];
            float nx = sample_x[idx];
            float ny = sample_y[idx];
            plot_map_.push_back(boost::make_tuple(x, y, nx - x, ny - y));
        }
    }

    // plan path
    auto [rx, ry] = dijkstraPlanning(sx, sy, gx, gy, road_map, sample_x, sample_y);

    // // final plot
    // gp << "plot '-' with vectors nohead title 'roadmap', '-' title 'obs', '-' with linespoints lw 5 title 'path'\n";
	// gp.send1d(plot_map_);
    // gp.send1d(boost::make_tuple(ox_, oy_));
    // gp.send1d(boost::make_tuple(rx, ry));


    return {rx, ry};
}


std::pair<std::vector<float>, std::vector<float>> 
    ProbabilisticRoadmap::samplePoints(float sx, float sy, float gx, float gy) {
    
    // get minmax of x and y 
    auto [min_x, max_x] = std::minmax_element(ox_.begin(), ox_.end());
    auto [min_y, max_y] = std::minmax_element(oy_.begin(), oy_.end());

    std::vector<float> sample_x;
    std::vector<float> sample_y;
    sample_x.reserve(sample_points_);
    sample_y.reserve(sample_points_);

    // random number generator
    std::random_device                  rand_dev;
    std::mt19937                        generator(rand_dev());
    std::uniform_real_distribution<float>  distr(0.0f, 1.0f);

    while (sample_x.size() <= sample_points_) {
        float tx = distr(generator) * (*max_x - *min_x) + *min_x;
        float ty = distr(generator) * (*max_y - *min_y) + *min_y;

        // look for closest point, see if we are more than robot radius away before pushing
        // these vectors will just have one index
        auto [indices, dists] = kdtree_.knnSearch(tx, ty, 1);
        
        if (std::sqrt(dists[0]) > robot_radius_) {
            sample_x.push_back(tx);
            sample_y.push_back(ty);
        }
    }

    // add start and goal
    sample_x.push_back(sx);
    sample_y.push_back(sy);
    sample_x.push_back(gx);
    sample_y.push_back(gy);

    return {sample_x, sample_y};
    
}


std::vector<std::vector<int>> 
        ProbabilisticRoadmap::generateRoadMap(std::vector<float>& sample_x, std::vector<float>& sample_y) {
    
    // for each sampled point, find n_knn_ number of neighbours
    size_t n_sample = sample_x.size();
    std::vector<std::vector<int>> road_map;
    road_map.reserve(n_sample);

    // generate kdtree from samples
    KDTree sample_kdtree(sample_x, sample_y);

    for (int i = 0; i < n_sample; ++i) {
        float x = sample_x[i];
        float y = sample_y[i];

        // sorts all other samples by distance
        auto [indices, dists] = sample_kdtree.knnSearch(x, y, n_sample);

        // try to find n_knn_ number of valid edges
        std::vector<int> edge_id;
        edge_id.reserve(n_knn_);
        for (size_t j = 0; j < indices.size(); ++j) {
            float nx = sample_x[indices[j]];
            float ny = sample_y[indices[j]];

            if (validEdge(x, y, nx, ny))
                edge_id.push_back(indices[j]);
            
            if (edge_id.size() >= n_knn_)
                break;
        }

        road_map.push_back(edge_id);
    }

    return road_map;
}

bool ProbabilisticRoadmap::validEdge(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    float yaw = std::atan2(dy, dx);
    float distance = std::hypot(dx, dy);

    // check edge length
    if (distance >= max_edge_len_) {
        return false;
    }
    
    // check if we collide along line
    float x = x1;
    float y = y1;
    float step_size = robot_radius_;
    int   n_step = std::round(distance/step_size);
    for (int i = 0; i < n_step; ++i) {
        auto [indices, dists] = kdtree_.knnSearch(x, y, 1);
        if (std::sqrt(dists[0]) < robot_radius_) {
            return false;
        }
        x += step_size * std::cos(yaw);
        y += step_size * std::sin(yaw);
    }

    // last check @ exactly p2, don't want rounding error
    auto [indices, dists] = kdtree_.knnSearch(x2, y2, 1);
    if (std::sqrt(dists[0]) <= robot_radius_) 
        return false;

    return true;
}



  std::pair<std::vector<float>, std::vector<float>> 
        ProbabilisticRoadmap::dijkstraPlanning(float sx, float sy, float gx, float gy, 
            std::vector<std::vector<int>>& road_map, 
            std::vector<float>& sample_x, std::vector<float>& sample_y) {

        Node start_node(sx, sy, 0, -1);
        Node goal_node(gx, gy, 0, -1);

        std::unordered_map<int, Node> open_set;
        std::unordered_map<int, Node> closed_set;

        // last two indices in road map belong to (sx, sy), (gx, gy) respectively
        open_set[road_map.size()-2] = start_node;

        // clear viz array
        expanded_x_.clear();
        expanded_y_.clear();

        bool found_path = true;
        while (true) {
            if (open_set.empty()) {
                std::cout << "Failed to find path!" << std::endl;
                found_path = false;
                break;
            }

            auto next_node = std::min_element(open_set.begin(), open_set.end(), 
            [&](const auto& p1, const auto& p2){
                return p1.second.cost < p2.second.cost;
            });
            int c_id = (*next_node).first;
            Node current = (*next_node).second;

            // plotting
            expanded_x_.push_back(current.x);
            expanded_y_.push_back(current.y);
            // gp << "plot '-' with vectors nohead title 'roadmap', '-' title 'obs', '-' title 'expanded'\n";
            // gp.send1d(plot_map_);
            // gp.send1d(boost::make_tuple(ox_, oy_));
            // gp.send1d(boost::make_tuple(expanded_x_, expanded_y_));

            if (c_id == (road_map.size()-1)) {
                std::cout << "Found goal!" << std::endl;
                goal_node.parent = current.parent;
                goal_node.cost = current.cost;
                break;
            }

            // remove from open set and add to closed set
            open_set.erase(c_id);
            closed_set[c_id] = current;

            // visit neighbours
            std::vector<int>& neighbours = road_map[c_id];
         
            for (int n_id : neighbours  ) {
                // continue if in closed
                if (closed_set.find(n_id) != closed_set.end()) 
                    continue;

                float dx = sample_x[n_id] - current.x;
                float dy = sample_y[n_id] - current.y;
                float distance = std::hypot(dx, dy);
                Node node(sample_x[n_id], sample_y[n_id], 
                            current.cost + distance, c_id);

                if (open_set.find(n_id) == open_set.end() || open_set[n_id].cost > node.cost)
                    open_set[n_id] = node;

            }
        }


        if (found_path == false)
            return {{}, {}};

        std::vector<float> rx;
        std::vector<float> ry;
        rx.push_back(goal_node.x);
        ry.push_back(goal_node.y);

        int parent_id = goal_node.parent;
        while (parent_id != -1) {
            auto& node = closed_set[parent_id];
            rx.push_back(node.x);
            ry.push_back(node.y);
            parent_id = node.parent;
        }

        return {rx, ry};
}

