#!/bin/bash

# Source your ROS 2 workspace (edit this if needed)
source ~/turtlebot3_ws/install/setup.bash

# Open each launch in a new terminal window
# gnome-terminal -- bash -c "ros2 launch semantic_navigation turtlebot3_small_house.launch.py; exec bash" &
gnome-terminal -- bash -c "ros2 launch slam_toolbox online_async_launch.py; exec bash" &
gnome-terminal -- bash -c "ros2 launch nav2_bringup navigation_launch.py; exec bash" &
gnome-terminal -- bash -c "ros2 launch turtlebot3_bringup rviz2.launch.py; exec bash" &
