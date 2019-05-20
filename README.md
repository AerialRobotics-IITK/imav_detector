# imav_detector
This repository hosts an object detection package aimed at the fulfilling the objectives of the IMAV 2019 problem statement.

## Dependencies
This package has the following dependencies:
- OpenCV (3.0 or higher)
- Eigen
- ROS Kinetic (stable, tested) with the following packages:

  - catkin
  - catkin_simple
  - roscpp
  - cmake_modules
  - message_generation
  - std_msgs
  - sensor_msgs
  - nav_msgs
  - geometry_msgs
  - eigen_conversions
  - cv_bridge
  - image_transport
  - tf

## Installation
Clone the repository into a suitable catkin workspace.
```bash
git clone https://github.com/ashwin2802/imav_detector.git
```
Build using either (preferably) `catkin build imav_detector` or `catkin_make` after ensuring all dependencies are met.  
