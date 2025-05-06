#!/bin/bash

# Immediately catch all errors
set -eo pipefail

sudo chown -R $(whoami) ~/ws
cd ~/ws

# Run rosdep installation
rosdep update
rosdep install --from-paths src --ignore-src -y

# Build the workspace
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install

# Source the workspace
echo "source ~/ws/install/setup.bash" >> ~/.bashrc
