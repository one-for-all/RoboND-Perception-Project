## Project: Perception Pick & Place

---


# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

[//]: # (Image References)
[raw point cloud]: ./misc_images/raw_point_cloud.png
[noise filtered 1]: ./misc_images/noise_filtered_1.png
[voxel grid downsampling]: ./misc_images/downsampled.png
[passthrough filtered]: ./misc_images/passthrough.png
[RANSAC plane filtered table]: ./misc_images/ransac_table.png
[RANSAC plane filtered objects]: ./misc_images/ransac_objects.png

[Clusters]: ./misc_images/clusters.png

[Confusion Matrix]: ./misc_images/confusion_matrix.png

[test scene 1 recognition]: ./misc_images/output_1.png
[test scene 2 recognition]: ./misc_images/output_2.png
[test scene 3 recognition]: ./misc_images/output_3.png
[challenge scene recognition]: ./misc_images/challenge.png

[//]: # (Hyperlink References)
[Statistical Outlier Removal]: http://pointclouds.org/documentation/tutorials/statistical_outlier.php
[SOR python doc]: http://strawlab.github.io/python-pcl/#pcl.StatisticalOutlierRemovalFilter
[Voxel Grid Downsampling]: http://pointclouds.org/documentation/tutorials/voxel_grid.php
[VGD python doc]: http://strawlab.github.io/python-pcl/#pcl.VoxelGridFilter
[Passthrough Filter]: http://pointclouds.org/documentation/tutorials/passthrough.php
[PF python doc]: http://strawlab.github.io/python-pcl/#pcl.PassThroughFilter
[RANSAC]: http://pointclouds.org/documentation/tutorials/random_sample_consensus.php
[RANSAC python doc]: http://strawlab.github.io/python-pcl/#pcl.Segmentation

[Euclidean Clustering]: http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

This document.

### Exercise 1, 2 and 3 pipeline implemented

The raw point cloud for test scene 1 is:

![Raw point cloud for test scene 1][raw point cloud]

#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

The first step of image processing was removing noise.   
This was done using the method of [Statistical Outlier Removal], with [python doc][SOR python doc].    
It basically ignores points that are too far from other points.

An example result after noise removal is:

![point cloud after noise removal][noise filtered 1]

Next, the point cloud was downsampled to reduce load for further processing, using [Voxel Grid Downsampling], with [python doc][VGD python doc].    
Like image downsampling, it divides the space into voxels, and the mean of points in each voxel is taken as the value of that voxel.

An example result is:

![point cloud after downsampling][voxel grid downsampling]

Then the point cloud was restricted to the area that we know beforehand where objects would appear.    
This was done with [Passthrough Filter], with [python doc][PF python doc].    
It removes points that are outside certain range in world frame.

After passthrough filtering:

![point cloud after passthrough filtering][passthrough filtered]

To separate objects from the table, [RANSAC] (Random Sample Consensus) was used, with [python doc][RANSAC python doc].    
RANSAC iteratively tries to fit points in the cloud to a predefined model, like spheres, planes and etc., and picks the best fit.    
Because the table in the environment can be modeled as a plane, RANSAC was used to fit it with a plane.

A example result of the filtered-out table:

![point cloud after RANSAC for table][RANSAC plane filtered table]

The remaining points which are the objects are:

![point cloud of objects][RANSAC plane filtered objects]

In the implementation, another more fine-grained noise filter (Statistical Outlier Removal) was applied to the objects to remove smaller noises.

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

To separate objects from each other, the point cloud was clustered into different clusters.    
Specifically, the points were clustered through [Euclidean Clustering], also called DBSCAN.    
It basically clusters close points together, while the ones that do not have enough close neighbors are considered outliers.

The point cloud after clustering looks like, (each cluster assigned a different color):

![point cloud after clustering][Clusters]

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

To classify point clouds of the objects, a machine learning model was constructed.     
The features are the color histogram of points in HSV space, concatenated with surface normal histogram.    
Around 10000 data samples have been collected.    
The machine learning model was SVM with linear kernel. Cross-validation accuracy of 0.94 +/- 0.01 was achieved.

The confusion matrix of the model is:

![Confusion Matrix]

The clusters obtained in the previous step was then fed to the machine learning model to get labels of the clusters.    
The result images are attached in the next section.

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

The results of object recognition are:

For test scene 1:

![test scene 1 recognition]

Result for test scene 2:

![test scene 2 recognition]

Result for test scene 3:

![test scene 3 recognition]

Messages that contain object centroids, the groups they belong, the name, the place to place them and test scene number are outputted into `output_1.yaml`, `output_2.yaml`, and `output_3.yaml`.


### Challenges

#### Perception Challenge
The same process did not apply very well to the challenge.world.

After some tweaking, including not doing passthrough filter, filtering more planes, the results were still not good.

The clusters look fairly cluttered:

![challenge scene recognition]

#### Motion Challenge

To build up a collision map, the robot was first instructed to turn left and right in order to see around.    
The point cloud that are used for collision map are the points that correspond to tables, but not objects.

Once the robot is finished building a collision map, the info of recognized objects that appear in the pick list are sent to the pick and place routine for action.

The execution of the entire process, however, is far from perfect.    
An example execution can be found at [this youtube link](https://youtu.be/XXh-U9o-l1w).

### Areas for improvements

The two challenges indeed exposed many areas for improvement.

1. The perception pipeline that worked for `test1.world`, `test2.world` and `test3.world`, was not generalized for all environments, as shown with `challenge.world`.    
In particular, we were not able to separate all the tables, background objects consistently from target projects, which result in very cluttered clusters.

2. The robot was not able to grasp the objects every time even if it had reached the correct pose.

3. When constructing the collision map, we did not know how to remove points from collision map to inform the robot of the new empty spaces.

4. In general, the entire process felt a bit sluggish, as seen when perception updates took some time to show. The VM uses 8 cores and 14 GB RAM.
