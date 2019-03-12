cmake_minimum_required(VERSION 2.8)
project( OpenCVTests )

find_package( OpenCV REQUIRED )
find_package( catkin REQUIRED COMPONENTS
    roscpp
)
include_directories( ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} )

add_executable( Display DisplayImage.cpp )
target_link_libraries( Display ${OpenCV_LIBS} )

add_executable( Webcam DisplayFromYI.cpp)
target_link_libraries( Webcam ${OpenCV_LIBS} )

add_executable( YI DisplayFromYI.cpp)
target_link_libraries( YI ${OpenCV_LIBS} )