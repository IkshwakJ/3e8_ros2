#!/usr/bin/env bash

# Author 	: Ikshwak
# Date 		: September 20 2025
# Description	: Partially builds the current ROS2 workspace.
# Note		:To skip building any packages, uncomment the corresponding lines below.
#             	Ensure that dependencies within the workspace are also commented out or removed.
#             	(package.xml in the meta package lists all dependencies; any additional dependencies
#             	must be considered.)
colcon build \
#	--package-skip 3e8_ros2_bringup \
#	--package-skip 3e8_ros2_description \
#	--package-skip 3e8_ros2_foxglove \
#	--package-skip 3e8_ros2_localization \
#	--package-skip 3e8_ros2_navigation \
#	--package-skip 3e8_ros2_system_tests \
#	--package-skip 3e8_ros2_teleop \
#	--package-skip 3e8_ros2_utils \
#	--package-skip gantry_controller \
#	--package-skip misc_motors \
#	--package-skip servo_controller \
#	--package-skip camera_driver \
#	--package-skip realsense_driver \
#	--package-skip servo_controller \
true

source install/setup.bash
