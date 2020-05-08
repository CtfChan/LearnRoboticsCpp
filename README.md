# LearnRoboticsCpp

This repository contains my implementations of classical robotics algorithms in C++. Inspiration drawn from [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) and [CppRobotics](https://github.com/onlytailei/CppRobotic). The CppRobotics repo was very good, but it used OpenCV to plot 2D graphs. I found the process of converting 2D points to pixel coordinates in OpenCV very tedious and it seems like a bit of a hack. This repo uses [gnuplot-iostream](https://github.com/dstahlke/gnuplot-iostream) instead for plotting which makes much prettier graphs than OpenCV and allows for us to easily make 3D plots.


## Table of Contents
* [Requirements](#requirements)
* [Build](#build)
* [Localization](#localization)
    * [Extended Kalman Filter](#extended-kalman-filter)
    * [Unscented Kalman Filter](#unscented-kalman-filter)
    * [Particle Filter](#particle-filter)
* [SLAM](#slam)
    * ICP 
    * EKF-SLAM 
    * FastSLAM
    * GraphSLAM
* [Path Planning](#path-planning)
    * [Dijkstra](#dijkstra)
    * [AStar](#astar)
    * [RRT](#rrt)
    * [RRTstar](#rrtstar)
    * [PRM](#prm)
    * [Potential Field](#potential-field)
    * [Quintic Polynomial](#quintic-polynomial)
    * [Cubic Spline](#cubic-spline)
    * [DWA](#dwa)
    * [Model Predictive Trajectory Generator](#model-predictive-trajectory-generator)
    * [State Lattice Planner](#state-lattice-planner)                 
    * DStar
* [Path Tracking](#path-tracking)
    * [Move to Pose](#move-to-pose)
    * [Stanely Control](#stanley-control)
    * Model Predictive Control
* [To do](#to-do)

## Requirments
- cmake
- opencv 3.3 
- Eigen 3
- Boost 1.4
- gnuplot
- ipoptd (this one is a pickle, TODO add instr.)
sudo apt-get install cppad



## Build
```console
$ mkdir build
$ cd build
$ cmake ../
$ make -j4
```





## Path Planning
### DWA 
![DWA Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/dwa.gif)

### Dijkstra 
![Dijkstra Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/dijkstra.gif)

### A* 
![A* Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/astar.gif)

### PRM 
![PRM Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/prm.gif)

### RRT 
![RRT Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/rrt.gif)

### RRTStar 
![RRTStar Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/rrt_star.gif)

### Potential Field 
![Potential Field Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/potential_field.gif)

### Quintic Polynomial
![Quintic Polynomial Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/quintic_polynomial.gif)
 
### Cubic Spline
![Cubic Spline Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/cubic_spline.gif)
 
### Model Predictive Trajectory Generation
![ Model Predictive Trajectory Generation](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/model_predictive_trajectory_generation.gif)
 

### State Lattice Planner
<p float="left">
  <img src="https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/uniform_sampling.png" width="300" />
  <img src="https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/biased_sampling.png" width="300" /> 
  <img src="https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/lane_sampling.png" width="300" />
</p>
Left to right: uniform, biased, lane sampling

## Path Tracking
### Move To Pose
![Move To Pose Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/move_to_pose.gif)

### Stanley Control
![Stanely Control Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/stanley_control.gif)


## Localization
### Extended Kalman Filter
![Extended Kalman Filter Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/ekf.gif)


### Unscented Kalman Filter
![Unscented Kalman Filter Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/ukf.gif)

We can see error ellipse in this demo is a much better approximation of the true distribution. EKF can be biased and inconsistent. 

### Particle Filter
![Particle Filter Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/particle_filter.gif)



## SLAM
### EKF-SLAM

### FastSLAM

### Graph Slam



## To Do
- [X] gnuplot scatter
- [X] gnuplot animation
- [x] dijkstra 
- [x] astar
- [x] prm
- [x] rrt
- [x] rrtstar
- [x] dwa
- [x] potential field
- [x] ekf
- [x] ukf
- [x] move to pose
- [x] cubic spline
- [x] quintic spline
- [x] stanley controller
- [x] model predictive trajectory generator
- [x] state lattice (do after above)
- [x] particle filtering
- [x] cleanup filters and consolidate common funct one header
- [ ] mpc
- [ ] use Pose2D instead of pose in a lot of places
- [ ] consolidate path trackers
- [ ] consolidate path planning algos
- [ ] EKFSLAM
- [ ] FAST SLAM
- [ ] Graph slam
- [ ] informed rrtstar
- [ ] batch informed rrtstar
- [ ] reed shepps path
- [ ] hybrid A*
- [ ] lidar to grip map
- [ ] frontier exploration 
- [ ] icp
- [ ] D*
- [ ] frenet frame
- [ ] voronoi -> need to find C++ impl. of voronoi sampling
- [ ] Improve astar and dij to use node rather than vector to save cost_to_come, also use open and close sets hash instead
- [ ] Plot start and goal point in prm, astar, dikj
- [ ] visualization of ekf, error ellipse
- [ ] gtest integration
- [ ] categorize into planning, localization, perception. hybrid approaches
- [ ] turn algos into ALG.hpp, ALG.cpp, example_ALG.cpp 
- [ ] decouple algo from gnuplot