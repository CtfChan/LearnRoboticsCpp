#pragma once

#include <iostream>
#include <vector>
#include <random>

#include <opencv2/opencv.hpp>
#include <opencv2/flann.hpp>

#include "gnuplot-iostream.h"


class KDTree {
private:
    cv::flann::Index tree_;
    cv::Mat_<float> features_;

public:
    KDTree(std::vector<float>& x_pts, std::vector<float>& y_pts) {
        cv::Mat_<float> features(0, 2);
        for (size_t i = 0; i < x_pts.size(); ++i) {
            cv::Mat row = (cv::Mat_<float>(1, 2) << x_pts[i], y_pts[i]);
            features.push_back(row);
        }    

        // this will segfault b/c it will call cv::flann::Index::release() after contructor 
        // finishes
        // tree_ = cv::flann::Index(features, cv::flann::KDTreeIndexParams(1));
        tree_.build(features, cv::flann::KDTreeIndexParams(1), cvflann::FLANN_DIST_L2); // squared euclidean

        features_ = features;
    }

    // returns vector of index and distances closest to (x, y)
    std::pair<std::vector<int>, std::vector<float>> knnSearch(float x, float y, int knn=1) {
        cv::Mat query = (cv::Mat_<float>(1, 2) << x, y);
        cv::Mat indices, dists; //neither assume type nor size here ! size will be(1, knn)

        tree_.knnSearch(query, indices, dists, knn, cv::flann::SearchParams(32));
     
        std::vector<int> indices_vec;
        std::vector<float> dists_vec;

        for (int i = 0; i < knn; ++i) {
            indices_vec.push_back( indices.at<int>(0, i));
            dists_vec.push_back( dists.at<float>(0, i));
        }

        return {indices_vec, dists_vec};
    }
};



struct Node {
    float x;
    float y;
    float cost;
    int parent;

    Node (float _x, float _y, float _cost, int _parent) : 
        x(_x), y(_y), cost(_cost), parent(_parent) {}

    Node () {}
};


class ProbabilisticRoadmap {
public:
    ProbabilisticRoadmap (std::vector<float>& ox, std::vector<float>& oy, float robot_radius, 
        int sample_points=500, int n_knn=10, float max_edge_len=30.0f);
    
    std::pair<std::vector<float>, std::vector<float>> 
        plan(float sx, float sy, float gx, float gy, Gnuplot& gp);


private:
    std::pair<std::vector<float>, std::vector<float>> 
        samplePoints(float sx, float sy, float gx, float gy);

    std::vector<std::vector<int>> 
        generateRoadMap(std::vector<float>& sample_x, std::vector<float>& sample_y);

    // takes in two points, determines whether or not edge between them is valid
    bool validEdge(float x1, float y1, float x2, float y2);

    std::pair<std::vector<float>, std::vector<float>> 
        dijkstraPlanning(float sx, float sy, float gx, float gy, 
            std::vector<std::vector<int>>& road_map, 
            std::vector<float>& sample_x, std::vector<float>& sample_y, Gnuplot& gp);

    std::vector<float> ox_;
    std::vector<float> oy_;

    float robot_radius_;

    int sample_points_;
    int n_knn_;
    float max_edge_len_; // in meters

    KDTree kdtree_;

    // for visualizations
    std::vector<boost::tuple<float, float, float, float> > plot_map_;
    std::vector<float> expanded_x_;
    std::vector<float> expanded_y_;

};


