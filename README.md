[![MIT License](https://img.shields.io/apm/l/atomic-design-ui.svg?)](https://github.com/leander-dsouza/atreus/blob/master/LICENSE) <img src="https://img.shields.io/badge/foxy-passing-green&style=plastic">

# Atreus

![Python 3](https://img.shields.io/badge/-Python-black?style=plastic&logo=Python)
![CMake](https://img.shields.io/badge/-CMake-064F8C?style=plastic&logo=CMake)
![ROS 2](https://img.shields.io/badge/-ROS_2-22314E?style=plastic&logo=ROS)

This package contains a mutlipurpose four-wheeled skid-steer drive robot equipped with sensors such as a Depth Camera, LiDAR, GPS, and an IMU.

Installation
------------

* Install all the required ROS related dependencies:

      rosdep install --from-paths src --ignore-src -r -y

* To install all python related dependencies:

	  pip install -r requirements.txt

* For basic bot simulation:

      ros2 launch atreus main.launch.py

### Optional

* To add an additional python dependency to the list, modify `requirements.in` and add a trailing library to the file. After which you need to install the following dependencies in order to use `pip-compile`:

		pip install pip-tools launchpadlib

* Finally use `pip-compile` to generate a `requirements.txt` file from `requirements.in`:

		pip-compile requirements.in

###### 💾 EOF