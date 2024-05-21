#!/bin/bash

PATH_TO_ROS2_BAG="/mnt/hgfs/cartella_condivisa/bag_18_05_24/antiorario_pomeriggio2ù/"

# Create a new tmux session named 'my_session' with bash shell
tmux new-session -d -s my_session bash

# Split the window vertically to create three terminals
tmux split-window -v -t my_session:0 bash
tmux split-window -v -t my_session:0 bash
tmux split-window -v -t my_session:0 bash

# Send commands to the first and second terminals
#tmux send-keys -t my_session:0.0 "cd ~/orbslam3-odometry  ; source install/local_setup.bash  ; ros2 launch orbslam3_odometry orbslam3-odometry_launch.py  " C-m
tmux send-keys -t my_session:0.1 "ros2 bag play $PATH_TO_ROS2_BAG -r 0.5 --topics /camera/right_image  /camera/left_image /imu/data /velodyne_points " C-m
tmux send-keys -t my_session:0.2 "cd ~/mmr-drive  ; source install/local_setup.bash  ; ros2 launch fast_lio fast_lio_launch.py " C-m
tmux send-keys -t my_session:0.3 "cd ~/orbslam3-odometry2/src/orbslam3_odometry/rviz_config ; rviz2 -d visualizza_orbslam_e_fastlio.rviz" C-m

# Attach to the session to view the terminals
tmux -2 attach-session -d -t my_session

