# Project Description:

This project was created to suit the telemetry needs of the Indian Rover Challenge 2020. This has two subparts in terms of options:
Teleoperation by time or by generation of waypoints. Both of these options include an initial travel on the designated path. 

### By Time: 

After the initial run is complete,  a file records the time duration and interval of the Twist message given to **atreus** over its course of traversal.
This is played back only when the rover *is exactly back where it started*. Therefore, without the use of any sensors, the rover is
able to traverse its initial path.

This has some inherent disadvantages. Synchronization and Surety of smooth traversal. The former being hindrances in getting to the initial position
and recording the exact time of input command. Latter, is to presume that the rover does not stuck in the surrounding that it has traversed before.

### By Waypoints:

Here, when the initial run is complete, **atreus** records GPS waypoints in a file. These points are then passed in a *gradient ascent*
algorithm* where these get approximated to a curve that ensures minimum hard turns for the rover to traverse to. These output points
then get written to an output file, and get followed.

This method makes use of two additional sensors - A GPS and an IMU. The benefit being, it does not require initial positioning
and does not rely on smooth terrain.


## Tutorial:

### 1) By Time: 

* Launch **atreus**:
        
        roslaunch atreus xacro.launch
        
* Connect your USB joystick and run the script:

        rosrun atreus teleoperation.py
       
* Now, traverse **atreus** into a path of your choice. After executing the custom path, kill this script. Get the rover to the initial position,
  then finally run the mimicing script.
  
       rosrun atreus mimic.py
  
### 2) By Waypoints: 

* Launch **atreus**:
        
        roslaunch atreus xacro.launch

* Run the dynamic gps plotter and driving node:

        rosrun atreus dynamic_plotting_with_file_write.py
        
* Drive **atreus** to any path you wish:

<img src="https://user-images.githubusercontent.com/45683974/77847055-c679cd80-71d7-11ea-9d0f-86c78b19ef05.gif" width="900" height="400">

* Run the gradient ascent curvefitting script:

       rosrun atreus gps_curve_fit.py

<img src="https://user-images.githubusercontent.com/45683974/77847147-73ece100-71d8-11ea-934b-838dce0120f0.png" width="900" height="450">

* Finally run the mimicing script:

       rosrun atreus key_mimic.py

#### Voilà! 

<img src="https://user-images.githubusercontent.com/45683974/77847372-3ab57080-71da-11ea-8e50-29e854db22bb.gif" width="900" height="400">


