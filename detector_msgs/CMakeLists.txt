cmake_minimum_required(VERSION 2.8.3)
project(detector_msgs)

find_package(catkin REQUIRED COMPONENTS
  catkin_simple
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  std_msgs
)

add_message_files(
  FILES
    BBox.msg
    BBPose.msg
    BBoxes.msg
    BBPoses.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    catkin_simple 
    geometry_msgs 
    message_generation 
    message_runtime 
    roscpp 
    std_msgs
)
