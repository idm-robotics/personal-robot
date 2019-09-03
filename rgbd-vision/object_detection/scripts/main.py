#!/usr/bin/env python

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from object_detection.msg import DetectedObjectArray
from yolo_detector import YOLODetector

VERBOSE = True

TOPIC_IN = "/camera/rgb/image_color"
TOPIC_OUT = "/object_detection/boxes"


class Publisher:
    def __init__(self):
        """Initialize ros publisher, ros subscriber"""
        # topic where we publish
        self.image_pub = rospy.Publisher(TOPIC_OUT, DetectedObjectArray, queue_size=1)
        self.bridge = CvBridge()

        # subscribed topic
        self.subscriber = rospy.Subscriber(TOPIC_IN, Image, self.callback, queue_size=1)
        if VERBOSE:
            print("subscribed to " + TOPIC_IN)

    def callback(self, ros_data):
        if VERBOSE:
            print('received image of type: "%s"' % ros_data.encoding)
        cv_image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
        object_detector = YOLODetector()
        msg = object_detector.detect(cv_image)
        self.image_pub.publish(msg)


def main():
    """Initializes and cleanup ros node"""
    Publisher()
    rospy.init_node('object_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
