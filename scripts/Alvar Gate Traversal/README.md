# Project Description:

This project serves its purpose as it was created for the Autonomous Task of University Rover Challenge 2020. It follows traversal
between a pair of Alvar Tags of dimensions - 20cm x 20cm. Mounted on a variable podium that ranges from 30cm - 100cm and is separated from
each other that varies from 2m -3m.

This is a custom traversal algorithm that satisfies the above need. The URDF and naturally the actual rover
has two cameras placed transversely to each other. This ensures smooth alignment, maximum range of detection and precise traversabililty.
The Alvar Tag detection is handled by the ROS package [ar_track_alvar](http://wiki.ros.org/ar_track_alvar).
The algorithm proceeds as follows:

* Listen for a frame appearance on either one of the cameras.
* Rotate left until the right side of the rover is parallel with the axis of the Alvar Tag.
  
  * **If** the left marker id is detected **then**
    * Move backward till the distance from the rover and the ARTags is rough the same.
  * **else**
    * Move forward till the distance from the rover and the ARTags is rough the same.

* Rotate right by 90° and then move forward and execute **correction algorithm**.
  
  * The **correction algorithm** ensures that the rover either sees both or none of the ARTags while moving forward, i.e
    rotates right if the left ARTag only is in frame and vice-versa.

## Tutorial:

* Launch **atreus**:

        roslaunch atreus xacro.launch
        
* Run the driving script. Note that detection has a range of 4m. Drive upto range.

        rosrun atreus key_drive.py
        
* Run the traversal script:

        rosrun atreus ar_traversal.py
        
#### Voilà!

<img src="https://user-images.githubusercontent.com/45683974/77855295-322a5d80-720d-11ea-8ee7-b3e4bbbbf39f.gif" width="900" height="450">


  


*

