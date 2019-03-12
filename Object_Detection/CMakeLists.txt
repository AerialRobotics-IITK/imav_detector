cmake_minimum_required(VERSION 2.8)
project( DisplayImage )
find_package( OpenCV REQUIRED )
find_package( catkin REQUIRED COMPONENTS
    roscpp
)
include_directories( ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} )
add_executable( DisplayImage DisplayImage.cpp )
target_link_libraries( DisplayImage ${OpenCV_LIBS} )
target_link_libraries( DisplayImage ${catkin_LIBRARIES})