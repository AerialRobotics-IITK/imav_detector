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
- picker   : a helper node used to determine the optimal HSV ranges for the objects of interest.


## Infograph
![rqt_graph goes here](rosgraph.png)


## Topics
Topics can be remapped to suit your use case using the `remap` tags in the `main.launch` file for the `detector_node` and the `picker.launch` file for the `picker` node.

### picker
  + Subscribed
    - /colour_image (sensor_msgs/Image)

### detector_node
  + Subscribed
    - /image        (sensor_msgs/Image)
    - /odometry     (nav_msgs/Odometry)   
  + Published
    - /bounding_boxes (detector/BBoxes)
    - /object_poses   (detector/BBPoses)
    - /undist_image   (sensor_msgs/Image)
    - /marked_image   (sensor_msgs/Image)
    
## Parameters
This section describes the parameters found in `detector.yaml` in more detail. The parameters can also be tuned while the node    is running using dynamic reconfigure - run `rosrun rqt_reconfigure rqt_reconfigure` on the terminal.

### camera
- is_rectified: specifies whether the image feed is undistorted or not
- translation : specifies coordinates of camera origin with reference to quad origin
- rotation : specifies the rotation matrix for quad to camera frame transformation

### box
- minSize: the minimum amount of pixels that a detected rectangle must have
- maxAreaIndex: specifies the error in the ratio of area calculated using corners of the rectangle and the area calculated using the number of pixels. Theoretically value should be 0. By tuning this one can control the size and number of holes due to noise inside a detected rectangle.
- maxEigenIndex: specifies the ratio of area calculated using PCA and the area calculated using the number of pixels. Theoretically value should be 1. By tuning this one can control the smoothness in the overall shape of the rectangle. 1.07 is the maximum permissible value, setting a value higher than this will correspond to shapes other than rectangles.
- maxDiagIndex: specifies the maximum difference in the lengths of the two diagonals of the detected rectangle. Theoretically value should be 0. By tuning this one can control the amount of distortion in the shape of the rectangle. Can be set as high as required.
- centreCorrectIndex: For rectangles that are partially in the frame, the actual centre and the calculated centre do not correspond. By tuning this one can control how much the calculated centre is pushed towards the edge of the frame. Higher the value, the closer the centre will be to the edge of the frame
- maxCentreDist: Specifies the maximum radius of the circle around the centre of the frame within which the store flag will activate if the rectangle centre comes within the circle.

### flags
- areaCheck: specifies whether to apply maxAreaIndex check. *Default: true*
- diagCheck: specifies whether to apply maxDiagIndex check. *Default: true*
- eigenCheck: specifies whether to apply maxEigenIndex check. *Default: true*
- sizeCheck: specifies whether to check ground-truth size of the detected rectangle. *Default: false*
- centreCorrect: specifies whether to apply centreCorrectIndex. *Default: false*
- debug: specifies whether to publish `marked_image` and `bounding_boxes`. *Default: true* when testing, *false* at runtime.
- verbose: specifies whether to publish pixel values when debug is on.  *Default: false*

### yellow, red, blue, orange
all H values are shifted 90 from their actual values
`h' = (h+90)%180`
- h_min: specifies min H value for the colour
- h_max: specifies max H value for the colour
- s_min: specifies min S value for the colour
- s_max: specifies max S value for the colour
- v_min: specifies min V value for the colour
- v_max: specifies max V value for the colour

### sizes
red, blue, yellow, orange - specify ground-truth sizes of red, blue, yellow and orange boxes respectively.
tolerance - specifies maximum allowed error in actual and calculated ground-truth size of rectangles
minHegiht - specifies the minimum height of the quad above which ground-truth size is checked if sizeCheck flag is set to true
