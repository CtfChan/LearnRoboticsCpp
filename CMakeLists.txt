cmake_minimum_required(VERSION 3.1)
project(CPP_ROBOTICS)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE Debug)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#############################
#Dependencies
#############################

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost 1.40.0 COMPONENTS filesystem system iostreams REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)


# Create library
## Declare a cpp library
add_library(${PROJECT_NAME}_core
    src/common.cpp
    # localization
    src/localization/filters.cpp
    src/localization/ekf.cpp
    src/localization/ukf.cpp
    src/localization/particle_filter.cpp
    
    # path tracking
    src/path_tracking/mpc.cpp
    src/path_tracking/move_to_pose.cpp
    src/path_tracking/stanley_control.cpp

    # path planning
    src/path_planning/potential_field.cpp
    src/path_planning/quintic_polynomial.cpp
    src/path_planning/trajectory_optimizer.cpp
    src/path_planning/state_lattice.cpp
    src/path_planning/cubic_spline.cpp
    src/path_planning/dwa.cpp
)





##############################
#PathPlanning
##############################

add_executable(dijkstra src/dijkstra.cpp)
target_link_libraries(dijkstra ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(astar src/astar.cpp)
target_link_libraries(astar ${OpenCV_LIBS} ${Boost_LIBRARIES} )

add_executable(prm src/prm.cpp) # use opencv for kdtree
target_link_libraries(prm ${OpenCV_LIBS} ${Boost_LIBRARIES} )

add_executable(rrt src/rrt.cpp)
target_link_libraries(rrt ${Boost_LIBRARIES} )

add_executable(rrt_star src/rrt_star.cpp)
target_link_libraries(rrt_star ${Boost_LIBRARIES} )

add_executable(potential_field examples/path_planning/example_potential_field.cpp)
target_link_libraries(potential_field ${Boost_LIBRARIES} ${PROJECT_NAME}_core)

add_executable(dwa examples/path_planning/example_dwa.cpp)
target_link_libraries(dwa ${Boost_LIBRARIES} ${PROJECT_NAME}_core)

add_executable(quintic_polynomial examples/path_planning/example_quintic_polynomial.cpp)
target_link_libraries(quintic_polynomial ${Boost_LIBRARIES} ${PROJECT_NAME}_core)

add_executable(cubic_spline examples/path_planning/example_cubic_spline.cpp)
target_link_libraries(cubic_spline ${Boost_LIBRARIES}  ${PROJECT_NAME}_core)

add_executable(trajectory_optimizer examples/path_planning/example_trajectory_optimizer.cpp)
target_link_libraries(trajectory_optimizer ${Boost_LIBRARIES} ${PROJECT_NAME}_core)

add_executable(lookup_table_generation examples/path_planning/example_lookup_table_generation.cpp)
target_link_libraries(lookup_table_generation ${Boost_LIBRARIES} ${PROJECT_NAME}_core)

add_executable(state_lattice examples/path_planning/example_state_lattice.cpp)
target_link_libraries(state_lattice ${Boost_LIBRARIES} ${PROJECT_NAME}_core)


##############################
#PathTracking
##############################

add_executable(move_to_pose examples/path_tracking/example_move_to_pose.cpp)
target_link_libraries(move_to_pose ${Boost_LIBRARIES} ${PROJECT_NAME}_core)

add_executable(stanley_control examples/path_tracking/example_stanley_control.cpp )
target_link_libraries(stanley_control ${Boost_LIBRARIES} ${PROJECT_NAME}_core)

add_executable(mpc examples/path_tracking/example_mpc.cpp)
target_link_libraries(mpc ${Boost_LIBRARIES} ${PROJECT_NAME}_core ipopt)


##############################
#Localization
##############################

add_executable(ekf examples/localization/example_ekf.cpp)
target_link_libraries(ekf ${Boost_LIBRARIES} ${PROJECT_NAME}_core)

add_executable(ukf examples/localization/example_ukf.cpp)
target_link_libraries(ukf ${Boost_LIBRARIES} ${PROJECT_NAME}_core)

add_executable(particle_filter examples/localization/example_particle_filter.cpp)
target_link_libraries(particle_filter ${Boost_LIBRARIES} ${PROJECT_NAME}_core)

