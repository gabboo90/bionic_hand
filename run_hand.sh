# !/bin/bash
# rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
# rqt &
ros2 launch launch_everything complete_hand.launch.py
