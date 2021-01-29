cmake_minimum_required(VERSION 3.17)
project(kalman_filters)

SET(CMAKE_CXX_FLAGS  "-Wall -Wextra")
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(kalman_filters main.cpp simple_kf.cpp)
