#!/usr/bin/env python

import cv2
import rospy
from cv_bridge import CvBridge
from object_detection.msg import DetectedObjectArray
from sensor_msgs.msg import Image

from yolo_detector import YOLODetector

TOPIC_IN = "/camera/rgb/image_rect_color"
TOPIC_OUT = "/object_detection/boxes"


class Publisher:
    def __init__(self):
        """Initialize ros publisher, ros subscriber"""
        # topic where we publish
        self.image_pub = rospy.Publisher(TOPIC_OUT, DetectedObjectArray, queue_size=1)
        self.bridge = CvBridge()
        self.object_detector = YOLODetector()

        # buff_size=2**24 because of: https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/
        self.subscriber = rospy.Subscriber(TOPIC_IN, Image, self.callback, queue_size=1, buff_size=2**24)
        rospy.loginfo("subscribed to " + TOPIC_IN)

    def callback(self, ros_data):
        start_time = rospy.Time.now()
        rospy.loginfo("Delay:%6.3f" % (start_time - ros_data.header.stamp).to_sec())

        rospy.loginfo('received image of type: "%s"' % ros_data.encoding)
        cv_image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")

        start_time = rospy.Time.now()
        msg = self.object_detector.detect(cv_image, ros_data.header)

        rospy.loginfo("Object detection time:%6.3f" % (rospy.Time.now() - start_time).to_sec())
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
