
# ORBSLAM3 Odometry Configuration Report

Here there is a small description about the orbslam3 configuration file. The `parameters' values` that are descripted here can be changed in the [yaml file](src/orbslam3_odometry/config/orbslam3_odometry.yaml).

## Subscription Topics

The node subscribes to the following topics and waits for data:
- **Camera Left**: The topic `/camera/left_image` provides the left camera images (<i>topic_camera_left</i>).
- **Camera Right**: The topic `/camera/right_image` provides the right camera images (<i>topic_camera_right</i>). 

Although ORB-SLAM3 provides a way to use also IMU data for the mapping/localization, we couldn't find a way to make it work properly. For this reason, this node won't subscribe to IMU. 

<!-- **IMU**: The topic `/imu/data` provides the Inertial Measurement Unit (IMU) data (<i>topic_imu</i>).  -->

## Publication Topics

The node publishes odometry data on the following topic:
- **ORBSLAM Odometry**: The topic `/Odometry/orbSlamOdom` publishes the odometry data calculated by ORBSLAM3 (<i>topic_orbslam_odometry</i>).



Also these headers values can be changed in the yaml file: 
-	Header Frame ID (`os_track`): This is the frame of reference for the published odometry data (<i>topic_header_frame_id</i>). To visualize both FastLio's and ORB-SLAM3's odometries in Rviz2, change this value to <i>fl_track</i>.
<!-- In RViz2, this frame is used as the reference coordinate system for visualizing the robot’s movement. All the odometry data is transformed into this frame before being displayed. -->
- Child Frame ID (`orbslam3`): This is the child frame ID in the header of the published odometry message (<i>topic_child_frame_id</i>). 
<!-- In the context of RViz2, this represents the moving frame attached to the robot. The transformation between the Header Frame ID and the Child Frame ID represents the robot’s pose (position and orientation) at a particular time. -->
In summary, these two topics are essential for correctly visualizing the robot’s odometry in RViz2. They ensure that the robot’s movement is displayed in the correct reference frame (os_track), and that the pose of the robot (orbslam3) is accurately represented over time.

## Visualization and File Paths

- **Pangolin Visualization**: This parameter (<i>pangolin_visualization</i>) controls the Pangolin visualizer, which is used to display the map created by ORBSLAM3 and the images passed with ROS2. If set to true, the visualizer is enabled. If set to false, the visualizer is disabled. Of course, for debug purpose, we advise to use `true` so there is an alternative view of what is going on inside ORB-SLAM3. But it should be setted to `false` after that, so the view won't be displayed. 
- **Path Vocabulary**: This parameter (<i>path_vocabulary</i>) specifies the full file path to the vocabulary of ORB_SLAM3. This vocabulary is required by ORB-SLAM3 and we always used the one provided by ORB-SLAM3. It can be found [here](src/orbslam3_odometry/config/ORBvoc.txt.tar.gz), but must be unzipped. Remember to provide the <b>full path</b> to the unzipped file.
- **Path YAML Settings**: This parameter (<i>path_yaml_settings</i>) specifies the full file path to the YAML settings for ORB_SLAM3. This file's content will be described below. Remember to provide the <b>full path</b> to the yaml file.

## Monocular or stereo?
The <i>system_mode</i> parameter specifies the mode of operation for ORB-SLAM3. The possible modes are `mono` or `stereo` <!--, and `stereo_imu` -->. After a lot of testing, we found that: 

- Stereo version works really good with stereo cameras (Zed2). The position and orientation is extremely good, compared to FastLio, considering also the costs of the components used by ORB-SLAM3 (relatively cheap) and FasLio's (expensive).
With Basler, stereo version works good. We tried turning the cameras in different positions and found that the best position is the following: cameras pointing outwards with an angle of 120° between them.
- Monocular version is the one we found better results (for Basler). It maps and relocalizes quite good. BUT bad news is monocular has no depth information, so the created map is not in scale with reality. We fixed this by multipling by a "scale factor" the returned position. 


### Monocular Version Parameters
For the monocular version of ORBSLAM3, the following parameters are used:
- **Is Camera Left**: This boolean parameter (<i>is_camera_left</i>) determines whether to subscribe to the left or right camera topic for images. If `true`, the left camera's topic is used. We advise to use the "internal" camera according to the direction of the circuit, so:
    - chose left camera if the direction is counter-clock-wise (anti-orario)
    - chose right otherwise

- **Scale Position Mono**: This parameter indicates the scale to multiply the calculated position. We use this factor because when we use a mono camera we have not a real scale factor due to impossible to determine the scene depth. We cannot predict how much it will be, but we tested that it's usually between 10 and 20, so we advise to use `15`. 

- **Degree Move Pose Mono**: This parameter (<i>degree_move_pose_mono</i>) specifies the degree to move the calculated orientation. We change the orientation to align it with FastLio (otherwise we would have oblique trajectory when the car is actually going straight).
    - If it is 0, the pose will not be changed. This should be used in  case camera is in the same direction as the forward axes.
    - If the cameras' support is the same as the one used for the test and cameras are turned with the same angles, we found that a value of `16` works good.  (TBD with new support)

Monocular takes images directly from the driver camera.

## How to get settings file content
Now, we will describe the content of the setting file that must be provided to ORB-SLAM. The full documentation of the file content can be found directly [here](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Calibration_Tutorial.pdf). 

Some examples of the settings file can be found in the [config directory](src/orbslam3_odometry/config/). 

### Common parameters
Starting from one of the example settings file, these parameters must be changed:
```YAML
# Camera frames per second. Can be obtained with ros2 topic hz /camera_topic
Camera.fps: 60

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 0

# ORB Extractor: Number of features per image. There is not a fixed value, somtimes it's necessary 
# to try with a few values, but we usually use between 1000 and 5000. We advise 3000. 
ORBextractor.nFeatures: 3000
```

### Monocular
For monocular, this parameters must be provided

```YAML
# "PinHole"  for Basler. "Rectified" for Zed2.
Camera.type: "PinHole"  

# Dimensions of the images
Camera.width: 1280      
Camera.height: 720

# Camera's intrinsics parameters. They are found after the calibration (es. with MatLab). They are the same used in the driver_camera node. For example:
Camera1.fx: 885.045449448846    
Camera1.fy: 888.752163653822
Camera1.cx: 650.293668844900
Camera1.cy: 303.574748229738

# These are the distortion parameters. Provide them only if the images that arrives from the driver_camera node are unrectified (so only if doRectify parameter in the driver_camera is false). Otherwise (so if the doRectify is true), you must provide them (you can also found them in the driver_camera's configuration file). p1 and p2 must be 0.0. For example:
Camera1.k1: 0.038229506627
Camera1.k2: -0.014204216000
Camera1.p1: 0.0
Camera1.p2: 0.0
```


### Stereo 
For the stereo version we provided some useful scripts to allow you to get the stereo parameters, used for rectification. (These are only required if Basler are used. If stereo camera are used, then the only required parameters are intrinsics value of one of the two cameras).  

To obtain stereo's rectification parameters, we created a few scripts. The steps that must be done are the following:

1. Get calibration images   
    
    TODO: da fare in modo meno macchinoso, senza la SHOW_reCTIFIED

    First thing to do is getting the pictures to calibrate cameras. The images can be obtained like this: 
    - In the  [yaml file](src/orbslam3_odometry/config/orbslam3_odometry.yaml), set to `true` the <i>just_take_picture</i> parameter 
    - In the [just_check_stereo_calibration.cpp](src/orbslam3_odometry/src/stereo/just_check_stereo_calibration.cpp) at line 150, change the path variable with the folder where calibration images will be saved. <b>Remember </b> to create this folder and two sub-directories (called <i>left</i> and <i>right</i>) before running the node.
    - Make sure `SHOW_RECTIFICATION` macro is NOT defined. (If it is, remove it and re-build the node). Tip: `roscd orbslam3_odometry && grep "SHOW_RECTIFICATION" -rn src/`
    - Run the node. Left and right images will be shown. When you are ready to take the picture, press the spacebar. The focus must be on the shown pictures, NOT on the terminal, so press with the mouse on the window with the shown pictures before taking the images.

    The information on how to put the checkboard in the pictures can be found online, but remember to: 
    - try to cover all the zone of the overlapped area 
    - take the picture only if the checkboard is entirely seen by both cameras


2. Calibrate cameras (es. with MatLab) and populate the settings file

    Once you have the pictures, use some tools to have the intrinsics and stereo parameters. Example of tools can be MatLab or Kalibr (not the missile :) ). 
    
    If you use MatLab, do as follow:
    - Use 'stereo calibration app' in MatLab: provide the pictures (first camera is the left) and the checkboard size. After the calibration, make sure the error is under 1 pixel (if it's not, remove the images with big errors and re-calibrate).
    - Once the calibration is okay, click on "Export" -> "Export stereo parameters in workspace"
    - You can now run [this script](src/orbslam3_odometry/from_matlab_to_opencv/from_matlab_to_opencv.m) in MatLab, to write on a file ('orbslam_parameters.txt') the calculated parameters. 
    - Put the content of 'orbslam_parameters.txt' directly in the yaml settings file.
    - Put in the yaml settings file also this parameter:
        ```YAML
        # Stereo baseline in meters (aka distance between camera left's and right's center)
        Stereo.b: 0.093
        ```
    
    If you use another tool, make sure to put in the setting yaml file: intrinsics cameras parameters, distortion cameras parameters, width and height of images, stereo matrix and camera type (PinHole/Rectified). More on this can be found directly [here](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Calibration_Tutorial.pdf), with an example on how to use Kalibr.

3. Show rectification

    To see the results of the stereo rectification with the given parameters, you can do as follow:

    - In the  [yaml file](src/orbslam3_odometry/config/orbslam3_odometry.yaml), set to `true` the <i>just_check_stereo_calibration</i> parameter. 
    - Make sure `SHOW_RECTIFICATION` macro IS defined. (If it's not, then add it and re-build the node). Tip: `roscd orbslam3_odometry && grep "SHOW_RECTIFICATION" -rn src/`
    - Run the node. Left and right rectified images will be shown, make sure the objects' horizontal position matches (if a point is on one of the red line on camera left, then it must be on the same line of the camera right). You can also visualize the disparity map

4. Run ORB-SLAM3
    You are now ready to run ORB-SLAM3! But don't forget to set to `false` the <i>just_check_stereo_calibration</i> parameter in the  [yaml file](src/orbslam3_odometry/config/orbslam3_odometry.yaml), 

## How we calculated the Odometry
### Monocular
The TrackMonocular function returns the position of the camera in the form of a quaternion. We take the quaternion and rotate it to have NED as the coordinate system since ORBSLAM uses EDN as the coordinate system. We also multiply the quaternions by a scaling factor (described above) because since we only have one camera, we do not have the actual scaling factor and therefore each run is calculated differently based on the median depth of the scene.

It is often necessary to apply a cut to input images because many times they focus on the wrong points. By cutting the image ORBSLAM3 focuses on cones 

## Stereo 
Very similar to Monocular, but we use trackStereo. We do not multiply quaternions because in this case we have a real scale factor. 

