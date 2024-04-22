# orbslam3-odometry
## Introduction
This repository is ROS2 wrapping to use ORB_SLAM3 and to publish ORB_SLAM3 Odometry on ROS2 topic.

## Prerequisites
I have tested on below version: 

* Ubuntu 20.04
* ROS2 Foxy
* Opencv 4.5.0

- Install related ROS2 package
``` bash
$ sudo apt install ros-$ROS_DISTRO-vision-opencv && sudo apt install ros-$ROS_DISTRO-message-filters
```
## How to build 
1. Clone this repository
``` bash
git clone https://github.com/Il-castor/orbslam3-odometry.git
```
2. Change this [line](src/orbslam3_odometry/CMakeLists.txt#L6) to your own `python site-packages` path
3. Change this [line](src/orbslam3_odometry/cmake/FindORB_SLAM3.cmake#L8) to your own `ORB_SLAM3` path

Build 
``` bash
colcon build --symlink-install
```
## Troubleshootings
If you cannot find `sophus/se3.hpp`:  
Go to your `ORB_SLAM3_ROOT_DIR` and install sophus library.

``` bash
$ cd ~/{ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus/build
$ sudo make install
```

## How to use
1. Source the workspace 
``` bash
$ source install/local_setup.bash
```

2. Modify parameters in `config/orbslam3_odometry.yaml` according to your system settings. Follow [these instructions](full_documentation.md). 

3. Launch 
``` bash
$ ros2 launch orbslam3_odometry orbslam3-odometry_launch.py
```


To stop the node press `ctrl-c` and this save ORB_SLAM3 statistics txt in your folder. 


