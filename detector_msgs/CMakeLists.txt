cmake_minimum_required(VERSION 2.8.3)
project(detector_msgs)

find_package(catkin REQUIRED)
find_package(catkin_simple REQUIRED)
catkin_simple()

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

cs_install()
cs_export()
