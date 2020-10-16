cmake_minimum_required(VERSION 3.10)
project(LEARN_ROBOTICS_CPP)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE Debug)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()


include_directories(include)


add_subdirectory (src)
add_subdirectory (tests)
add_subdirectory (examples)


