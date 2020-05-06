#pragma once

#include "common.hpp"

#include "trajectory_optimizer.hpp"


std::vector<Pose2D> sampleStates();

std::vector<Pose2D> calculateUniformPolarStates();

std::vector<Pose2D> calculateBiasedPolarStates();


/**
 * @brief 
 * 
 * @param l_center lane lateral positioning
 * @param l_heading lane heading
 * @param l_width lane width
 * @param v_width vehicle width
 * @param d longitudal position
 * @param nxy sampling number
 * @return std::vector<Pose2D> 
 */
std::vector<Pose2D> calculateLaneStates(float l_center, float l_heading, 
                            float l_width, float v_width, float d, size_t nxy);



std::pair<std::vector<TableEntry>, Arrow> generatePaths(TrajectoryOptimizer& optim, 
                                    std::vector<TableEntry>& lookup,
                                    std::vector<Pose2D>& states, float k0);


