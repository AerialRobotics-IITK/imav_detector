# imav_detector
This repository hosts an object detection package aimed at the fulfilling the objectives of the IMAV 2019 problem statement.

## References
This package was written with major references to [this](http://mrs.felk.cvut.cz/data/papers/mbzirc-vision2017.pdf) paper and its [code](https://github.com/gestom/MBZIRC_2017_vision.git).

## Dependencies
This package has the following dependencies:
- OpenCV (3.0 or higher)
- Eigen
- ROS Kinetic (stable, tested) with the following packages:

  - catkin
  - [catkin_simple](https://github.com/catkin/catkin_simple)
  - roscpp
  - [usb_cam](https://github.com/ros-drivers/usb_cam.git) (for obtaining images from a camera connected via USB)
  - [cmake_modules](https://github.com/ros/cmake_modules)
  - message_generation (for creating and using custom messages)
  - std_msgs
  - sensor_msgs
  - nav_msgs
  - geometry_msgs
  - [eigen_conversions](https://github.com/ros/geometry) (Eigen compatibility with ROS)
  - [cv_bridge](https://github.com/ros-perception/vision_opencv) (OpenCV compatibility with ROS)
  - image_transport
  - [tf](https://github.com/ros/geometry)


## Installation
If you haven't already, create a catkin workspace.
```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin config -DCMAKE_BUILD_TYPE=Release
catkin init
```

Clone the repository into the source folder.
```shell
cd ~/catkin_ws/src
git clone https://github.com/ashwin2802/imav_detector.git
```
Build using either (preferably) `catkin build imav_detector` or `catkin_make` after ensuring all dependencies are met.  


## Nodes
- detector_node : Detects rectangles in the image stream and calculates their locations in ground truth.
- thresh_node   : Performs image processing on the RGB input image to convert it into a binary image.


## Infograph
![rqt_graph goes here](rosgraph.png)


## Topics
Topics can be remapped to suit your use case using the `remap` tags in the `main.launch` file.

### thresh_node
  + Subscribed
    - /colour_image (sensor_msgs/Image)
  + Published
    - /threshold_image (sensor_msgs/Image)

### detector_node
  + Subscribed
    - /image        (sensor_msgs/Image)
    - /odometry     (nav_msgs/Odometry)   
  + Published
    - /bounding_boxes (detector/BBoxes)
    - /object_poses   (detector/BBPoses)
    - /undist_image   (sensor_msgs/Image)
    - /marked_image   (sensor_msgs/Image)
