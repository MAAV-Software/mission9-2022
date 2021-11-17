cmake_minimum_required(VERSION 3.20)
project(MAAV_Path_Generator)

set(CMAKE_CXX_STANDARD 14)
include_directories("/Eigen")
add_executable(MAAV_Path_Generator main.cpp)
