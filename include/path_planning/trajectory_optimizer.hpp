
#include <Eigen/Eigen>
#include "common.hpp"
#include "gnuplot-iostream.h"

// For lookuptable
struct TableEntry {
  float x;
  float y;
  float yaw;
  float s;
  float km;
  float kf;
};
TableEntry findClosestEntry(const std::vector<TableEntry> &table, float x,
                            float y, float yaw);

// assume quadratic param, this is our p vector
struct TrajectoryParam {
  float s;   // path length
  float km;  // steer in the middle
  float kf;  // final steer
};

// assumes constant velocity
// designed to optimize trajectory for bicycle model robot to nearby points
class TrajectoryOptimizer {
 public:
  TrajectoryOptimizer(BicycleModelRobot &state, size_t max_iter, float cost_th,
                      TrajectoryParam &delta_params, float ds = 0.1f,
                      float min_alpha = 1.0f, float max_alpha = 2.0f,
                      float d_alpha = 0.5f);

  // takes in target pose, initial guess of params, initial steer (this is
  // fixed) outputs first trajectory that fits within cost limits
  Pose2DTrajectory optimizeTrajectory(Pose2D &target_pose,
                                      TrajectoryParam params, float k0,
                                      Gnuplot &gp);

  // this one does not plot and modifies params by reference
  Pose2DTrajectory optimizeTrajectory(Pose2D &target_pose,
                                      TrajectoryParam &params, float k0);

 private:
  // use quadratic interpolation to generate a trajectory given some (s, km, kf)
  Pose2DTrajectory generateTrajectory(TrajectoryParam &params, float k0);

  Pose2D generateFinalState(TrajectoryParam &params, float k0);

  Eigen::MatrixXf calculateJacobian(Pose2D &target_pose,
                                    TrajectoryParam &params, float k0);

  Eigen::Vector3f calculatePoseErrorVector(const Pose2D &p1, const Pose2D p2);

  float selectLearningParam(Pose2D &target_pose, TrajectoryParam params,
                            float k0, Eigen::Vector3f &dp);

  BicycleModelRobot init_state_;
  size_t max_iter_;
  float cost_th_;
  TrajectoryParam delta_params_;
  float ds_;

  // learing rate
  float min_alpha_, max_alpha_, d_alpha_;
};