S-PTAM is a Stereo SLAM system able to compute the camera trajectory in real-time. It heavily exploits the parallel nature of the SLAM problem, separating the time-constrained pose estimation from less pressing matters such as map building and refinement tasks. On the other hand, the stereo setting allows to reconstruct a metric 3D map for each frame of stereo images, improving the accuracy of the mapping process with respect to monocular SLAM and avoiding the well-known bootstrapping problem. Also, the real scale of the environment is an essential feature for robots which have to interact with their surrounding workspace.


<a href="http://www.youtube.com/watch?feature=player_embedded&v=kq9DG5PQ2k8
" target="_blank"><img src="http://img.youtube.com/vi/kq9DG5PQ2k8/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>  
(Click the image to redirect to S-PTAM video)

## Related Publications:
[1] Taihú Pire, Thomas Fischer, Javier Civera, Pablo De Cristóforis and Julio Jacobo Berlles.  
**Stereo Parallel Tracking and Mapping for Robot Localization**  
Proc. of The International Conference on Intelligent Robots and Systems (IROS) (Accepted), Hamburg, Germany, 2015.


# License

S-PTAM is released under GPLv3 license.

For a closed-source version of S-PTAM for commercial purposes, please contact the authors.

If you use S-PTAM in an academic work, please cite:

@inproceedings{pireIROS15,  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  title={{Stereo Parallel Tracking and Mapping for robot localization}},  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  author={Pire, Taih{\'u} and Fischer, Thomas and Civera, Javier and De Crist{\'oforis}, Pablo and Jacobo berlles, Julio},  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  booktitle={Proc. of The International Conference on Intelligent Robots and Systems (IROS) (Accepted)},   
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  year={2015}  
 }

# Disclaimer
This site and the code provided here are under active development. Even though we try to only release working high quality code, this version might still contain some issues. Please use it with caution.

# Prerequisites (dependencies)

## ROS

We have tested S-PTAM in Ubuntu 14.04 with ROS Indigo.

To install ROS (indigo) use the following command:

`sudo apt-get install ros-indigo-desktop`


## g2o

Install [g2o](https://openslam.org/g2o.html) library from the source code provided in  

`svn co https://svn.openslam.org/data/svn/g2o`  

Note: the g2o ROS package `ros-indigo-libg2o` in ubuntu 14.04 reduces notoriusly the performance of S-PTAM.

## PCL

`sudo apt-get install ros-indigo-pcl-ros`

# Installation

`git clone git@github.com:lrse/sptam.git`

# Compilation

`catkin_make --pkg sptam -DSHOW_TRACKED_FRAMES=ON`

## CMAKE flags

SHOW_TRACKED_FRAMES=([ON|OFF], default: OFF)  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Show the tracked frames by S-PTAM. Set it OFF to improve S-PTAM performance.


# ROS Package

# Turorials

We provide some examples of how to run S-PTAM with the most popular stereo datasets

## KITTI dataset

1. Download the KITTI rosbag [kitti_00.bag](http://www6.in.tum.de/~kloses/rvc/kitti_bags/kitti_00.bag) provided in   [KITTI rosbag files](http://www6.in.tum.de/~kloses/rvc/kitti_bags/)  

2. Uncompress the dataset  

	`rosbag decompress kitti_00.bag`  

3. Set use_sim_time ros variable true  

    `rosparam set use_sim_time true`  

4.  Play the dataset  

	`rosbag play --clock kitti_00.bag`  
	
	(When S-PTAM run with the flag SHOW_TRACKED_FRAMES=ON the performance is reduced notoriusly).
5. Run sptam using the kitti.launch  

    `roslaunch sptam kitti.launch`  


## MIT Stata Center dataset

1. Download the MIT Stata Center rosbag [2012-01-27-07-37-01.bag](http://infinity.csail.mit.edu/data/2012/2012-01-27-07-37-01.bag) provided in   [MIT Stata Center Web Page](http://projects.csail.mit.edu/stata/downloads.php)  

2. Set use_sim_time ros variable true  

    `rosparam set use_sim_time true`  

3.  Play the dataset  

	`rosbag play --clock 2012-01-27-07-37-01.bag -s 302.5 -u 87`  
	 
	(Here we are running the [part 3 of the sequence](http://infinity.csail.mit.edu/data/2012/utilities/2012-01-27-07-37-01) where ground-truth was provided that is why the bag file start from a different timestamp)

4. Run sptam using the mit.launch  

    `roslaunch sptam mit.launch`  


## Indoor Level 7 S-Block dataset

1. Download the Level7 rosbag [level07_20_05_12_trunc.bag (3747 Frame Subset)](https://wiki.qut.edu.au/display/cyphy/Indoor+Level+7+S-Block+Dataset) provided in [Indoor Level 7 S-Block Dataset Web Page](https://wiki.qut.edu.au/display/cyphy/Indoor+Level+7+S-Block+Dataset)  

2. Set use_sim_time ros variable true  

    `rosparam set use_sim_time true`  

3.  Play the dataset  

	`rosbag play --clock level7_truncated.bag`  
	 

4. Run sptam using the level7.launch  

    `roslaunch sptam level7.launch`  


## Node Information

### Subscribed Topics

Camera topics should provide undistorted and stereo-rectified images. Consider using the [image_proc](http://wiki.ros.org/stereo_image_proc) node.

/stereo/left/image\_rect ([sensor\_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Undistorted and stereo-rectified image stream from the left camera.

/stereo/left/camera\_info ([sensor\_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Left camera metadata.

/stereo/right/image\_rect ([sensor\_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Undistorted and stereo-rectified image stream from the right camera.

/stereo/right/camera\_info ([sensor\_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Right camera metadata.

### Published Topics

point_cloud ([sensor\_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Sparse mapped point cloud used for tracking.

camera_pose ([geometry\_msgs::PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Current camera pose computed after tracking.

<!--
path ([nav\_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Path navigated by the robot, following the poses computed by the tracker.
-->

### Parameters

~use\_odometry (bool, default: false)  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
 Replace decay velocity motion model by odometry.

~odom\_frame (string, default: "odom")  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Reference frame for odometry messages, if used.

~base\_link\_frame: (string, default: "base\_link")  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Reference frame for the robot base.

~camera\_frame: (string, default: "camera")  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Reference frame for the left camera, used to get left camera pose from tf.

~approximate_sync: (bool, default: false)  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Whether to use approximate synchronization. Set to true if the left and right Cameras do not produce exactly synced timestamps.




~FeatureDetector/Name: (string, default: "GFTT")  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Follows OpenCV convention.




~DescriptorExtractor/Name: (string, default: "BRIEF")  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Follows OpenCV convention. 




~DescriptorMatcher/Name: 'BruteForce-Hamming'  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Follows OpenCV convention.  

~DescriptorMatcher/crossCheck: false  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Follows OpenCV convention.

~MatchingCellSize: (int, default: 15)  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
To match the map points with images features, each frame is divided in squares cells of fixed size. MatchingCellSize define the size of each cell.   

~MatchingNeighborhood: (int, default: 1)  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Cells' neighborhood around the point

~MatchingDistance: (double, default: 25.0)  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Descriptor distance. Use a non-fractional value when hamming distance is used. Use Fractional value when L1/L2 norm is used.  

~EpipolarDistance: (double, default: 0.0)  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Distance in rows from the epipolar line used to find stereo matches.

<!--
~KeyFrameDistance: (double, default: 0.0)  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;


~FramesBetweenKeyFrames: (int, default: 0)    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
-->

~FrustumNearPlaneDist: (double, default: 0.1)  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Frustum (Field of View) near plane.

~FrustumFarPlaneDist: (double, default: 1000.0)  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Frustum (Field of View) far plane.


<!--
# Camera parameters file

A YAML format file containing camera parameters.

Sample file:

	image_width: 1241
	image_height: 376

	camera_matrix: !!opencv-matrix
	  rows: 3
	  cols: 3
	  dt: d
	  data: [ 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1 ]

	baseline: 0.5371657188644

 
# S-PTAM configuration parameters file

A YAML format file containing S-PTAM configuration parameters.

Sample file:

    FeatureDetector:

      Name: 'GFTT'
      nfeatures: 2000
      minDistance: 15.0
      qualityLevel: 0.01
      useHarrisDetector: false

    DescriptorExtractor:
      Name: 'BRIEF'
      bytes: 32

    DescriptorMatcher:
  	  Name: 'BruteForce-Hamming'
      crossCheck: false

	MatchingCellSize: 15
	MatchingNeighborhood: 1
	MatchingDistance: 25
	EpipolarDistance: 0.0
	KeyFrameDistance: 0.0
	FramesBetweenKeyFrames: 0
	FrustumNearPlaneDist: 0.1
	FrustumFarPlaneDist: 100.0
-->