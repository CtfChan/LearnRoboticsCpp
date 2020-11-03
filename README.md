# LearnRoboticsCpp

This repository contains my implementations of classical robotics algorithms in C++. Inspiration drawn from [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) and [CppRobotics](https://github.com/onlytailei/CppRobotic). The CppRobotics repo was very good, but it used OpenCV to plot 2D graphs. I found the process of converting 2D points to pixel coordinates in OpenCV very tedious and it seems like a bit of a hack. This repo uses [gnuplot-iostream](https://github.com/dstahlke/gnuplot-iostream) instead for plotting which makes much prettier graphs than OpenCV and allows for us to easily make 3D plots.

Some of these implementations will have a tutorial attached to it. It's still a work in progress.

## Table of Contents
* [Requirements](#requirements)
* [Dependencies Installation](#dependency-installation)
* [Build](#build)
* [Localization](#localization)
    * [Extended Kalman Filter](#extended-kalman-filter)
    * [Unscented Kalman Filter](#unscented-kalman-filter)
    * [Particle Filter](#particle-filter)
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
* [Path Tracking](#path-tracking)
    * [Move to Pose](#move-to-pose)
    * [Stanely Control](#stanley-control)
    * [Model Predictive Control](#model-predictive-control)

## Requirments
Tested on Ubuntu 18.04
- cmake
- opencv 3.3 (for KD tree in PRM)
- Eigen 3
- Boost 1.4 (for gnuplot-iostream)
- gnuplot
- ipoptd (this one is a pickle, [install tips borrowed from Udacity](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/install_Ipopt_CppAD.md) )
- cppad


## Running with Docker
```
$ sudo docker build -f Dockerfile -t learn_robotics_cpp .
$ sudo docker run -it learn_robotics_cpp
```

## Dependencies Installation
1. apt installs
```
sudo apt update
sudo apt install build-essential
sudo apt install cmake
sudo apt install gnuplot
sudo apt install libboost-all-dev
sudo apt install libopencv-dev python3-opencv
sudo apt install libeigen3-dev
sudo apt install cppad
```

2. ipoptd install
```
sudo apt install gfortran
sudo apt install unzip
```

You probably want to `cd` into another directory like `Downloads` for this folder
```
wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip && unzip Ipopt-3.12.7.zip && rm Ipopt-3.12.7.zip
```
Call `install_ipopt.sh` in this repo with sudo and the source directory as the first argument (e.g. `sudo bash install_ipopt.sh ~/Ipopt-3.12.7`)

3. `Xming` (for Windows Subsystem Linux users only)
Need to install xming for gnuplot visualization: https://sourceforge.net/projects/xming/.
Set display to `gnuplot` with: `$ export DISPLAY=:0 gnuplot`.


4. Install Ceres

## Build
```console
$ git clone https://github.com/CtfChan/LearnRoboticsCpp.git
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

[Explanation](https://ctfchan.github.io/blog/quintic-polynomials)

### Cubic Spline
![Cubic Spline Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/cubic_spline.gif)

[Explanation](https://ctfchan.github.io/blog/cubic-splines)

### Model Predictive Trajectory Generation
![ Model Predictive Trajectory Generation](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/model_predictive_trajectory_generation.gif)

[Explanation](https://ctfchan.github.io/blog/trajectory-generator)

### State Lattice Planner
<p float="left">
  <img src="https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/uniform_sampling.png" width="300" />
  <img src="https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/biased_sampling.png" width="300" />
  <img src="https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/lane_sampling.png" width="300" />
</p>
Left to right: uniform, biased, lane sampling

[Tutorial](https://ctfchan.github.io/blog/state-lattice)

## Path Tracking
### Move To Pose
![Move To Pose Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/move_to_pose.gif)

### Stanley Control
![Stanely Control Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/stanley_control.gif)

### Model Predictive Control
![Model Predictive Control Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/mpc.gif)

[Tutorial](https://ctfchan.github.io/blog/mpc)

## Localization
### Extended Kalman Filter
![Extended Kalman Filter Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/ekf.gif)

[Tutorial](https://ctfchan.github.io/blog/ekf)

### Unscented Kalman Filter
![Unscented Kalman Filter Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/ukf.gif)

We can see error ellipse in this demo is a much better approximation of the true distribution. EKF can be biased and inconsistent.

[Tutorial](https://ctfchan.github.io/blog/ukf)

### Particle Filter
![Particle Filter Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/particle_filter.gif)




