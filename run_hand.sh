#!/bin/bash

# ROS2 Setup laden (passe den Pfad an deine ROS2-Installation an)
source /opt/ros/jazzy/setup.bash  # oder /opt/ros/foxy/setup.bash je nach Version

# rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
# rqt &
ros2 launch launch_everything complete_hand.launch.py