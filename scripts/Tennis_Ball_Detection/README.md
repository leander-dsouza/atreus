# Project Description:

This project along with 'Dynamic Traversal with Obstacle Avoidance' was the final task in the prototype phase of Mars Rover Manipal.
It involves detection of a tennis ball with standard dimensions through OpenCV after reaching the goal coordinates.

This involves conversion into a **HSV** colour space for easy mask extraction. Sequentially, follows a series of **Gaussian Blur** and
**Morphological filters** to eliminate noise.
An iteration through the contours is done to extract the **Bounding Box** and the **Min Area Rectangle**
for aspect ratio extraction based on certain permissible values. 

**Hough Circles** have been applied inside the Minimum Area Rectangle (after aspect ratio extraction) for ball extraction.
When in the use of sunlight, **Gamma Correction filter** can be applied.

## Tutorial:

* Launch **atreus**:

      roslaunch atreus xacro.launch
      
* Choose 'Official_Tennis_Ball' model from the insert tab in Gazebo and place it in **atreus'** view:

![Screenshot from 2020-03-30 00-43-32](https://user-images.githubusercontent.com/45683974/77858385-d8cc2980-7220-11ea-8dd7-db2748e5f834.png)

* Finally run the detection script and set gamma = 1.0 using the toolbar:

      rosrun atreus ball_detection_taskphase.py

#### Voilà!

![Screenshot from 2020-03-30 00-46-30](https://user-images.githubusercontent.com/45683974/77858440-319bc200-7221-11ea-89d5-ac38ead7163a.png)
