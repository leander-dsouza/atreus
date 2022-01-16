[![MIT License](https://img.shields.io/apm/l/atomic-design-ui.svg?)](https://github.com/leander-dsouza/atreus/blob/master/LICENSE) <img src="https://img.shields.io/badge/noetic-passing-green&style=plastic">

# Atreus

![Python](https://img.shields.io/badge/-Python-black?style=plastic&logo=Python)
![C++](https://img.shields.io/badge/-C%2B%2B-00599C?style=plastic&logo=C%2B%2B)
![CMake](https://img.shields.io/badge/-CMake-064F8C?style=plastic&logo=CMake)
![ROS](https://img.shields.io/badge/-ROS-22314E?style=plastic&logo=ROS)

This package contains a mutlipurpose four-wheeled skid-steer drive robot equipped with sensors such as IntelRealSense D435 Depth Camera, LiDAR, Ultrasonics, GPS, IMU, Magnetometer and a pair of cameras.

Functionality 
------------

* Obstacle Avoidance with soft turning using pcl processing (passthrough + voxel + RANSAC + euclidean clustering + centroidal analysis).

* Detection of slope angle and ditch depth by using a sonar panel.

* Dynamic traversal using GPS and IMU to reach desired location along with sonar obstacle avoidance.

* Ability of localisation by using two independent EKFs.

* Teleoperation with respect to waypoints/time by path smoothening.

* Alvar Gate Traversal.

* Tennis Ball Detection.

* Custom Global Planner

    <img src="https://user-images.githubusercontent.com/45683974/77653435-ad81da00-6f95-11ea-88cb-1e7cbcd500f9.gif" width="900" height="500">

* Traffic Light Model

    <img src="https://user-images.githubusercontent.com/45683974/77582804-fab66b00-6f05-11ea-915e-847d5defb0b9.gif" width="900" height="500">

Dependencies 
------------

* [**ar_track_alvar**](https://github.com/mojin-robotics/ar_track_alvar)
* [**realsense_gazebo_plugin**](https://github.com/SyrianSpock/realsense_gazebo_plugin)
* [**gzsatellite**](https://github.com/plusk01/gzsatellite)
* [**aws_robomaker_small_warehouse_world**](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)


Installation 
------------

* Install all the required ROS related dependencies:

      rosdep install --from-paths src --ignore-src -r -y 

* To install all python related dependencies:

	  pip install -r requirements.txt
        
* To install all Mapviz related dependencies:

      ./mapviz_install.sh 
    
* For basic bot simulation:

      roslaunch atreus xacro.launch

### Optional

* To add an additional python dependency to the list, modify `requirements.in` and add a trailing library to the file. After which you need to install the following dependencies in order to use `pip-compile`:

		pip install pip-tools launchpadlib

* Finally use `pip-compile` to generate a `requirements.txt` file from `requirements.in`:

		pip-compile requirements.in


Errors
------------
*  Run the following command if the traffic light does not glow red: 

       ./traffic_light_dep_fix.sh 

* Run the following commands if the map cache in mapviz does not appear.<br/>Based on the errors displayed, correspondingly run the commands and then launch mapviz:

1) **ERROR: 1** or **ERROR: 203**

       sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy

2) **ERROR: 2** or **ERROR: 401**

       sudo docker container ls
  
       sudo docker stop {container id}

       sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy

3) **ERROR: 302**

       Configured correctly. Poor internet connection.


###### 💾 EOF