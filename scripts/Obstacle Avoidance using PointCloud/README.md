# Project Description:

This project makes use of pcl filters in aim at better processing, segmentation and clustering of incoming obstacles.
Passthrough provides dimensional cutoff, Voxel downsamples, Random Sample Consensus segments ground plane (Eps = 0°) and finally 
clustering follows the Euclidean approach.

All of this pre-processing provides with a better estimate of the centroid of the resultant pointcloud. Therefore, the abscissa
and the applicate axes of this centroid outputs the magnitude and direction of the yield vector. This is fed into the Twist of **atreus**.

Before and after **Passthrough**:

<img src="https://user-images.githubusercontent.com/45683974/77789674-77be1d80-7089-11ea-87bc-05cde09255c8.jpg">

Before and after **Voxel**:

<img src="https://user-images.githubusercontent.com/45683974/77789738-9ae8cd00-7089-11ea-8fa5-c99854f1036c.jpg">

Before and after **RanSAC**:

<img src="https://user-images.githubusercontent.com/45683974/77789547-3af22680-7089-11ea-9751-4b92fc3e3467.jpg">

## Tutorial:

* Launch **atreus**:

      roslaunch atreus xacro.launch

* Finally, run the directional obstacle avoidance code:

      rosrun atreus pcl_obstacle_avoidance.py
      
**Voilà!**

<img src="https://user-images.githubusercontent.com/45683974/77792066-b7870400-708d-11ea-86b1-149c54c27932.gif" width="900" height="450">


      
 
