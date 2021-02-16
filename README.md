# Perception
![Alt text](https://github.com/ros-workshop/perception/blob/master/apriltagrobots_overlay.jpg)

## April Tags

In this session we will be using USB cameras to detect [April](https://april.eecs.umich.edu/software/apriltag.html) tags and view the precise 3D position, orientation, and identity of the tags relative to the camera in RVIZ.

There are limited USB cameras, so we will be using the built in camera in your laptop where we can and supplied [rosbag](http://wiki.ros.org/Bags) files. 

You will be supplied with printed April tags at the session as well as checkerboards if you would like to perform camera calibration.

![Alt text](https://github.com/ros-workshop/perception/blob/master/tags_rviz.png )
<img src="https://github.com/ros-workshop/perception/blob/master/tagformats_web.png" width="300" title="">

## April Tag Exercise
Use your USB camera to detect an apriltag! We are using [tag36h11](https://www.dotproduct3d.com/uploads/8/5/1/1/85115558/apriltags1-20.pdf), these are provided.

 * Install the required packages, you may use whatever you prefer for running your camera but we will be using ros-melodic-apriltag-ros for detection
 <br/>
 
<details>
<summary>Click for a hint</summary>

`sudo apt install ros-melodic-apriltag-ros ros-melodic-usb-cam` is a quick and easy node to get your webcam going
</details>

<br/>

 * Create a launch file to start your usb camera, check that it is working with RVIZ
 * Calibrate your camera with a checkerboard and http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
 * Add the apriltag node and configure it (tip: you need to configure the node to subscribe to your camera publisher and add your apriltag to the config file)
 
 <br/>
  <details>
 
 
<summary>Click for a hint</summary>
 You may have issues with your camera being uncalibrated, check the error output of your console.
 For the hardware provided the calibration is available here:
 
https://github.com/ros-workshop/perception/blob/master/ost.yaml

https://github.com/ros-workshop/perception/blob/master/ost.txt

</details>
<br/>

 * Add a static tf from the map to camera at the height your camera is above the ground (a tape measure is provided)
 
 * View the detection in RVIZ (the image)
 
 * View the detection in RVIZ (the tf)
 
 <br/>

## Stretch goal
 
If the above is completed in the session we will be supplying [Hokuyo](https://www.hokuyo-aut.jp/search/single.php?serial=166) LiDARs, depth camera and usb cameras, and we will set up an face detection and position estimation system. We will then build our own python node to output the location of detected people.

This stretch goal assumes you are familiar with:
* Python 
* [Subscribing and Publishing Topics](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
* Rosbag playing

There are limited Hokuyos available so [rosbag](http://wiki.ros.org/Bags) files will be supplied for students to work with until they are ready to test their software.

If you are using either the USB Cameras or the Hokuyo LiDAR you need to have a USB Type A socket.
 
 ## LiDAR Face detection Exercise

 * Install: If you're using a hokuyo, install the urg_node package otherwise install the appropriate node for your LiDAR / depth sensor
 * Create a launch file that starts the LiDAR and the usb camera (or use the provided rosbag) *hint: can you access the device as your user?*
 <br/>
<details>
<summary>Click for a hint</summary>

Usually googling the device and ROS will bring up the driver.
 
 If you cannot open the LiDAR:
https://answers.ros.org/question/286646/error-connecting-to-hokuyo-could-not-open-serial-hokuyo/

Other resources:
https://answers.ros.org/question/251060/how-to-use-an-usb-hokuyo-laserscanner-in-ros-kinetic/

</details>
<br/>

 * View the outputs of the camera and LiDAR / Depth Sensor in RVIZ
 * Write a python node that subscribes to the image and LiDAR topics
 * Use OpenCV to perform face detection on the image
 * Calculate the angles of the LiDAR that overlap the cameras vision (the lidar is 270 degrees, single layer). This can be done roughly, even with your hand if you are using the hardware. *It would be better to do this in the urg_node configuration, limiting the scan angle, rather than selecting a subset of the array... but each will work.*
 * Take the center of detected objects and roughtly figure out the angle they are at 
 * Using that angle to get the LiDAR distnace
 * Output the object type and distance as text
 
  ### Exercise stretch
 * Publish the tf of the detected object
 
 #### Hokuyu in RVIZ
  ![Alt text](https://github.com/ros-workshop/perception/blob/master/LiDAR_Hok_RVIZ.png)

