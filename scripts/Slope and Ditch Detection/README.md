# Project Description:

This package was created in reference to this [research paper](https://www.hindawi.com/journals/js/2013/643815/). It describes a
method on how to obtain slope angle and ditch depth without the use of image processing by using a panel made up of 4 sonar sensors.

<img src="https://user-images.githubusercontent.com/45683974/77821713-58b79e00-7112-11ea-8cee-ec4454555993.jpg" width="900" height="650">

This project implements a custom-based URDF to aid the said above. The working of this project credits to the research paper mentioned.
Follow the below tutorials to implement the same:

## Tutorial:

* Launch **atreus**:
      
      roslaunch atreus xacro.launch

* Run the driving node:

      rosrun atreus key_drive.py
      
* Finally run the object feasibility script:

      rosrun atreus ultrasonic_object_feasibility.py 

* Note that the models are available in the world launched for slope and ditch feasibility. Model names:- *Ramp* and *Ditch*      
 
 <img src="https://user-images.githubusercontent.com/45683974/77821381-e5149180-710f-11ea-8326-a48ea194cc40.gif" width="900" height="400">
 
 <img src="https://user-images.githubusercontent.com/45683974/77821683-2e65e080-7112-11ea-86a3-80594b53654b.gif" width="900" height="450">
