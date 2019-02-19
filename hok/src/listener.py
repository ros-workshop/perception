#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from dnn_detect.msg import DetectedObjectArray
import math


last_range = None
min_r = 640
max_r = 720
total_r = 726

def callback(msg):
    global last_range
    global min_r
    global max_r
    last_range = msg.ranges[min_r:max_r]
   

"""    class_name: "diningtable"
    confidence: 0.98298740387
    x_min: 327.0
    x_max: 638.0
    y_min: 356.0
    y_max: 479.0"""

def callback_dnn(msg):
    found = ""
    image_width = 640
    i = 0
    for obj in msg.objects: 
        i = i + 1
        x_middle = (obj.x_min + obj.x_max) / 2
        perc_across = 1 - (x_middle / 640)
        rospy.loginfo("perc %s", perc_across)
        distance = None
        loc = 0
        if last_range is not None:
            loc = int(perc_across*len(last_range))
            distance = last_range[loc]
        found = obj.class_name + " at x position: " + str(x_middle) + ", at a distance: " + str(distance) + "m"
        rospy.loginfo("DNN Callback %s", found)



def listener():
    print("Starting hok node")
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("scan", LaserScan, callback)
    rospy.Subscriber("/dnn_objects", DetectedObjectArray, callback_dnn)

    rospy.spin()

if __name__ == '__main__':
    listener()
