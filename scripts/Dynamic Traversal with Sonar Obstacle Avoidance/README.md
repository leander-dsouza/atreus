# Project Description:

This project was created to simulate the final task of our time in Mars Rover Manipal Taskphase. 
This rover consists of 4 sonar sensors, 2 placed transversely to each other. This placement was to ensure that even when the immediate greedy approach 
was satisfied, the other sonar pair would facilitate object passage.

Additionally, consisting of an IMU and GPS, this rover can traverse to any given GPS coordinate, while dynamically maintaining obstacle threshold.

## Tutorial:

* Launch **atreus**:
        
        roslaunch atreus xacro.launch
 
* Run the required script:

        rosrun atreus taskphase_bot.py

* It will prompt for a goal location. Type diligently (example goal location given).

        ENTER GOAL GPS COORDINATES - 38.4194166147 -110.781989913

#### Voilà!

<img src="https://user-images.githubusercontent.com/45683974/77827968-d85b6200-713e-11ea-8466-b840e852e41c.gif" width="900" height="400">
