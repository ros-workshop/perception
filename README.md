# Perception

## April Tags

In this session we will be using USB cameras to detect [April](https://april.eecs.umich.edu/software/apriltag.html) tags and view the precise 3D position, orientation, and identity of the tags relative to the camera in RVIZ.

There are limited USB cameras, so we will be using the built in camera in your laptop where we can and supplied [rosbag](http://wiki.ros.org/Bags) files. 

You will be supplied with printed April tags at the session as well as checkerboards if you would like to perform camera calibration.

## Stretch goal

If the above is completed in the session we will be supplying [Hokuyo](https://www.hokuyo-aut.jp/search/single.php?serial=166) LiDARs and usb cameras, and we will set up an object detection and position estimation system. We will then build our own python node to output the location of detected people.

This stretch goal assumes you are familiar with:
* Python 
* [Subscribing and Publishing Topics](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
* Rosbag playing

There are limited Hokuyos available (~5) so [rosbag](http://wiki.ros.org/Bags) files will be supplied for students to work with until they are ready to test their software.

If you are using either the USB Cameras or the Hokuyo LiDAR you need to have a USB Type A socket.

![Alt text](https://github.com/ros-workshop/perception/blob/master/apriltagrobots_overlay.jpg)
![Alt text](https://github.com/ros-workshop/perception/blob/master/tagformats_web.png)

#### April Tag Exercie
Use your USB camera to detect an apriltag!

 * Install the required packages, you may use whatever you prefer for running your camera but we will be using apriltag2_ros for detection
 * Create a launch file to start your usb camera, check that it is working with RVIZ
 * Add the apriltag node and configure it (tip: you need to configure the node to subscribe to your camera publisher and add your apriltag to the config file)
 * Add a static tf from the map to camera at the height your camera is above the ground
 * View the detection in RVIZ (the image)
 * View the detection in RVIZ (the tf)
 
 #### LiDAR Object detection Exercise
 ![Alt text](https://github.com/ros-workshop/perception/blob/master/DNN_detect.png)
 * Install the dnn_detect and urg_node packages
 * Create a launch file that starts the LiDAR and the usb camera (or use the provided rosbag)
 * View the outputs of the camera and LiDAR in RVIZ
 * Add the dnn detect node and view in RVIZ
 * Write a python node that subscribes to the dnn detect and LiDAR nodes
 * Calculate the angles of the LiDAR that overlap the cameras vision (the lidar is 270 degrees, single layer). This can be done roughly, even with your hand if you are using the hardware.
 * Take the center of detected objects and roughtly figure out the angle they are at. 
 * Using that angle to get the LiDAR distnace
 * Output the object type and distance
  ![Alt text](https://github.com/ros-workshop/perception/blob/master/LiDAR_Hok_RVIZ.png)

