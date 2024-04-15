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

2. Modify parameters in `config/orbslam3_odometry.yaml` according to your system settings

3. Launch 
``` bash
$ ros2 launch orbslam3_odometry orbslam3-odometry_launch.py
```


To stop the node press `ctrl-c` and this save ORB_SLAM3 statistics txt in your folder. 


## How to get stereo parameters
Use MATLAB to stereo-calibrate the cameras. Once it's done, export the stereo parameters result in the workspase. Use [this MATLAB script](src/orbslam3_odometry/from_matlab_to_opencv/from_matlab_to_opencv.m) to convert parameters to OpenCV and write them in a file. 

Take the content of the generated <i>orbslam_parameters.txt</i> file and copy it in the settings yaml settings file. Set just_check_stereo_calibration to true to view the results of the stereo-calibration, then reset it to false to start ORB-SLAM3 in the next execution.




### TODO
La stereo non permette la subscription best effort delle camere. va fixato come nella stereo-inertial
Controlla se sono stati fatti 
Aggiungi nodo ros con orbslam
Stesso formato di:
	https://github.com/mmr-driverless/mmr-drive/tree/master/src/1_perception/fusion


Possibili configurazioni:
- Vocabolario
- Path allo yaml
- Enable/Disable pangolin
- Topics pubblicazione e sottoscrizione
- Fixed frame 
- Salvare odometry su file
- modify launch 
