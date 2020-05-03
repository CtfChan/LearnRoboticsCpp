# LearnRoboticsCpp

This repository contains my implementations of classical robotics algorithms in C++. Inspiration drawn from [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) and [CppRobotics](https://github.com/onlytailei/CppRobotic). I used gnuplot-iostream (CITE) to plot the results.

## Requirment
- cmake
- opencv 3.3 
- Eigen 3
- Boost 1.4
- gnuplot

## Build
```console
$ mkdir build
$ cd build
$ cmake ../
$ make -j4
```


## Path Planning
### DWA Algorithm
![DWA Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/dwa.gif)

### Dijkstra Algorithm
![Dijkstra Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/dijkstra.gif)

### A* Algorithm
![A* Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/astar.gif)

### PRM Algorithm
![PRM Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/prm.gif)

### RRT Algorithm
![RRT Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/rrt.gif)

### RRTStar Algorithm
![RRTStar Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/rrt_star.gif)

### Potential Field Algorithm
![Potential Field Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/potential_field.gif)

### Quintic Polynomial
![Quintic Polynomial Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/quintic_polynomial.gif)
 
## Path Tracking
### Move To Pose
![Move To Pose Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/move_to_pose.gif)



## Localization
### EKF (Extended Kalman Filter)
![Extended Kalman Filte Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/ekf.gif)


### UKF (Unscented Kalman Filter)
![Unscented Kalman Filte Demo](https://github.com/CtfChan/LearnRoboticsCppGifs/blob/master/animations/ukf.gif)

We can see error ellipse in this demo is a much better approximation of the true distribution. EKF can be biased and inconsistent. 

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
- [ ] cubic spline
- [ ] quintic spline
- [ ] particle filtering
- [ ] stanley controller
- [ ] mpc
- [ ] state lattice (do after mpc)
- [ ] informed rrtstar
- [ ] batch informed rrtstar
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