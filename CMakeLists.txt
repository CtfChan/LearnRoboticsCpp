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

add_executable(dwa src/dwa.cpp)
target_link_libraries(dwa ${Boost_LIBRARIES})

add_executable(potential_field src/potential_field.cpp)
target_link_libraries(potential_field ${Boost_LIBRARIES})


##############################
#Localization
##############################

add_executable(ekf src/ekf.cpp)
target_link_libraries(ekf ${Boost_LIBRARIES})

add_executable(ukf src/ukf.cpp)
target_link_libraries(ukf ${Boost_LIBRARIES})

# add_executable(demo src/demo.cpp)
# target_link_libraries(demo ${OpenCV_LIBS} ${Boost_LIBRARIES} )


# add_executable(scatter src/scatter.cpp)
# target_link_libraries(scatter ${OpenCV_LIBS} ${Boost_LIBRARIES} )
