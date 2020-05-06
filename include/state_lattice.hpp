#pragma once

#include "common.hpp"

#include "trajectory_optimizer.hpp"

/**
 * @brief 
 * 
 * @param angle_samples this parameter influences density of samples at each angle
 * @param a_min 
 * @param a_max 
 * @param d 
 * @param p_max 
 * @param p_min 
 * @param nh 
 * @return std::vector<Pose2D> 
 */
std::vector<Pose2D> sampleStates(std::vector<float>& angle_samples, float a_min,
                 float a_max, float d, float p_max, float p_min, size_t nh);

/**
 * @brief Calculate uniform states.
 * 
 * @param nxy number of positions to sample (np in paper)
 * @param nh number of headings to sample
 * @param d distance of terminal states
 * @param a_min position sampling min angle
 * @param a_max position sampling max angle
 * @param p_min heading sampling min angle
 * @param p_max heading sampling max angle
 * @return std::vector<Pose2D> 
 */
std::vector<Pose2D> calculateUniformPolarStates(size_t nxy, size_t nh, float d,
                        float a_min, float a_max, float p_min, float p_max);



/**
 * @brief 
 * 
 * @param goal_angle 
 * @param ns 
 * @param nxy 
 * @param nh 
 * @param d 
 * @param a_min 
 * @param a_max 
 * @param p_min 
 * @param p_max 
 * @return std::vector<Pose2D> 
 */
std::vector<Pose2D> calculateBiasedPolarStates(float goal_angle, int ns, int nxy,
                                   int nh, int d,
                                   float a_min, float a_max,
                                   float p_min, float p_max);


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


