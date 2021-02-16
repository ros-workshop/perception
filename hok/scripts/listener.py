#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cv2


class FaceDetector(object):

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image)
        self.bridge = CvBridge()
        self.face_cascade = None
        self.face_cascade = cv2.CascadeClassifier(rospy.get_param("~haar_config"))
        self.last_range = None
        self.min_r = 640
        self.max_r = 720
        self.total_r = 726
        self.image_width = 640
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # Convert into grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # Detect faces
            faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)

            for (x, y, w, h) in faces:
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                if self.last_range:
                    x_middle = float(self.image_width - ((x + x + w) / 2))
                    perc_across = (x_middle / self.image_width)
                    distance = None
                    if self.last_range is not None:
                        loc = int(perc_across * len(self.last_range))
                        distance = self.last_range[loc]
                    rospy.loginfo(
                        "Face at pixel postion: %3d, percentage accross image: %2.2f, at a distance: %2.3f" %
                        (x_middle, perc_across, distance)
                    )

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            pass

    def laser_callback(self, msg):
        self.last_range = msg.ranges[self.min_r:self.max_r]


if __name__ == '__main__':
    rospy.init_node('face_detector', anonymous=True)
    face_detector = FaceDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

