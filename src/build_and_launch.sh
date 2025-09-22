#!/usr/bin/env bash

# Author	: Ari
# Date		: August 1 2025
# Description	: Build and launches the current ROS2 workspace using the file launch.py in launch/robot_bringup.

colcon build
source install/setup.bash
ros2 launch robot_bringup launch.py
