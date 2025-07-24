[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Lint](https://github.com/leander-dsouza/atreus/actions/workflows/lint.yml/badge.svg)

# Atreus for ROS 2 - Jazzy

![Python 3](https://img.shields.io/badge/-Python-black?style=plastic&logo=Python)
![CMake](https://img.shields.io/badge/-CMake-064F8C?style=plastic&logo=CMake)
![ROS 2](https://img.shields.io/badge/-ROS_2-22314E?style=plastic&logo=ROS)

This package contains a mutlipurpose four-wheeled skid-steer drive robot equipped with sensors such as a Depth Camera, LiDAR, GPS, and an IMU.

Installation
------------

### 1. Dev Container (Recommended)

* Install the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension for Visual Studio Code.

* Open the repository in Visual Studio Code and select `Reopen in Container` from the Command Palette (`Ctrl+Shift+P`).
This will build the development container with all the necessary dependencies.

### 2. Native Installation

* Clone the repository for responsible for driving the robot using arrow keys:

   ```bash
   cd $ROS_WS/src/
   git clone git@github.com:leander-dsouza/key_teleop_ros.git
   ```

* Install all the required ROS related dependencies:

   ```bash
   rosdep install --from-paths $ROS_WS/src --ignore-src -r -y
   ```

* Build the packages:

   ```bash
   colcon build --symlink-install --packages-select atreus key_teleop_ros
   ```

Tutorial
--------

### 1. RViz Joint Visualization

* Launch the robot in RViz:

   ```bash
   ros2 launch atreus rviz.launch.py
   ```
   https://github.com/user-attachments/assets/d9053cb4-9933-4115-bb04-ae64e0fe399b

### 2. Teleoperation in Gazebo

* Launch the robot in Gazebo:

   ```bash
   ros2 launch atreus gazebo.launch.py
   ```

* Open a new terminal and run the teleoperation node:

   ```bash
   ros2 run key_teleop_ros key_drive
   ```

   Use the arrow keys to control the robot's movement.

   <p align="center">
      <img width="510" height="107" alt="driving_script_interface" src="https://github.com/user-attachments/assets/561d433e-1ad9-4e9e-9f08-7ec0fb330eb0" />
   </p>

   https://github.com/user-attachments/assets/cd9df2df-2087-4dc8-8ce1-d5b526d30321

### 3. Mapping

* Launch the robot in Gazebo:

   ```bash
   ros2 launch atreus gazebo.launch.py
   ```

* Open another terminal and run the mapping node:

   ```bash
   ros2 launch atreus slam_toolbox.launch.py
   ```

   This will start the mapping process using **SLAM Toolbox**.

* Open a new terminal and run the teleoperation node:

   ```bash
   ros2 run key_teleop_ros key_drive
   ```

   This will allow you to map the environment by driving the robot around.

   https://github.com/user-attachments/assets/b92657ef-2855-4d6b-8c25-e220f16a3816


* In order to save the map, open a new terminal and run the following command:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f my_map
   ```

   This will save the map to the current directory.


###### 💾 EOF
