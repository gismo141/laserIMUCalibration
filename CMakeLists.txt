cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project(laserIMUCalibration)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(VERSION 0.1-alpha)

add_definitions(-std=c++11)

add_subdirectory(documentation) # doxygen documentation
add_subdirectory(src)