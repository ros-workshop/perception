# Perception

One of the key features that differentiate robots from simple machines is the execution or adaptation of actions based upon observations of the scene around it.
In this lesson, we will be exploring one of the more ubiquitous elements of robotic perception, detection of a fiducial marker.
We will also be touching on complex detection of a human face in a stretch goal.

![Alt text](./resources/apriltagrobots_overlay.jpg)

## April Tags

[April tags](https://april.eecs.umich.edu/software/apriltag.html) are a form of fiducial (derived from the latin word for trust) developed in one of the robotics departments at the University of Michigan.
Fiducials are physical markers that can be localised in 3D with respect to a camera that's viewing them in 2D.

![Alt text](./resources/tags_rviz.png)

## Exercise

In this session, we will be using simple and cheap cameras to detect and view the precise position and orientation of the tags relative to the camera.  You can either use your laptop's USB camera, or work with a prerecorded [rosbag](http://wiki.ros.org/Bags) file.

### Option 1: USB camera

Either use your laptop's camera, or ask an instructor to borrow a USB camera. Look for the camera calibration boards and April Tags that have been provided around the workshop. We will be using the April Tag group [tag36h11](https://www.dotproduct3d.com/uploads/8/5/1/1/85115558/apriltags1-20.pdf).

A ROS driver is required for your camera; most USB cameras and in-built laptop camera can be run with the `usb_cam` package. To calibrate your camera we'll use the `camera_calibration` package.

Install these packages: 

```bash
sudo apt install ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-apriltag-ros \
    ros-$ROS_DISTRO-camera-calibration
```




### Preperation
There are two alternatives for this exercise, depending if you have a camera on your laptop:

<h4>Alternate 1:</h4>
You will need a driver for the camera you will be using, and the Apriltag package.
Most USB cameras and in-built laptop camera can be run with the `usb_cam` package.
We will also be using a particular ros package for calibrating your sensor; `camera_calibration`.

```bash
sudo apt install ros-$ROS_DISTRO-usb-cam ros-$ROS_DISTRO-apriltag-ros ros-$ROS_DISTRO-camera-calibration
```
<b>Steps:</b>
<br/>
- [ ] Create a launch file to start your usb camera, check that it is working with RVIZ
- [ ] Calibrate your camera with a checkerboard and http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

<h4>Alternate 2:</h4>
Use the rosbag april_tag.bag, this will be provided by your mentor. Here you will playpack a recorded camera.

You will need the Apriltag package.
```bash
ros-$ROS_DISTRO-apriltag-ros
```
<b>Steps:</b>
<br/>
- [ ] Create a launch file to start your bag playback including using the loop command, check that it is working with RVIZ

### April Tag Exercise

Use your USB camera to detect an apriltag!
We are using [tag36h11](https://www.dotproduct3d.com/uploads/8/5/1/1/85115558/apriltags1-20.pdf), these are provided as A4 paper tags for alternative 1 and is already in the image of the bag.

The exercise is layed out below.
<br/><b>Steps:</b>
<br/>
- [ ] Add the apriltag node to your launch file and configure it (tip: you need to configure the node to subscribe to your camera publisher and add your apriltag to the config file)
- [ ] Add a static tf from the map to camera at the height your camera is above the ground (a tape measure is provided)
- [ ] View the detection in RVIZ (the image)
- [ ] View the detection in RVIZ (the tf)

## Stretch goal
 
If the above is completed in the session we will be supplying a bag (face_detection.bag) containing a LiDAR, usb camera, and we will set up an face detection and position estimation system. We will then build our own python node to output the location of detected people.

This stretch goal assumes you are familiar with:
* Python 
* [Subscribing and Publishing Topics](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
* Rosbag playing

### LIDAR face detection

- [ ] Create a launch file that starts the rosbag (it makes it easier if you loop the bag)
- [ ] View the outputs of the camera and LiDAR / Depth Sensor in RVIZ
- [ ] Write a python node that subscribes to the image and LiDAR topics
- [ ] Use OpenCV to perform face detection on the image
- [ ] Calculate the angles of the LiDAR that overlap the cameras vision (the lidar is 270 degrees, single layer).
  - This can be done roughly.
- [ ] Take the center of detected objects and roughtly figure out the angle they are at 
- [ ] Using that angle to get the LiDAR distance
- [ ] Output the object type and distance as text

<br/>

### Further stretch

- [ ] Publish the tf of the detected object
