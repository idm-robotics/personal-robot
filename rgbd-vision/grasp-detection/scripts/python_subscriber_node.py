#!/usr/bin/env python

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

GRAB_INDEX = 0.15


def callback(data):
    # rospy.loginfo(rospy.get_name()+" %s ",data.data)
    bridge = CvBridge()

    def draw_sidepoints(left, top, right, bottom, grab):
        left_point = (int(left + (right - left) * grab), (top + bottom) // 2)
        right_point = (int(right - (right - left) * grab), (top + bottom) // 2)
        cv2.circle(image, left_point, 2, (0, 255, 0), 2)
        cv2.circle(image, right_point, 2, (0, 255, 0), 2)

    def draw_box(a, b):
        cv2.rectangle(image, a, b, (255, 0, 0), 2)

    image = None
    try:
        # image = bridge.imgmsg_to_cv2(data, desired_encoding='mono16')
        # data.encoding = "bgr8"
        image = bridge.imgmsg_to_cv2(data, "bgr8")

        x = 255
        y = 265
        x1 = 350
        y1 = 360

        left_top_corner = (x, y)
        right_bottom_corner = (x1, y1)

        draw_box(left_top_corner, right_bottom_corner)
        draw_sidepoints(*left_top_corner, *right_bottom_corner, GRAB_INDEX)

    except CvBridgeError as e:
        print(e)

    # cv2.imwrite("image_try", image)
    cv2.imshow('image', image)
    cv2.waitKey(2)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("camera/rgb/image_color", Image, callback)
    # declares that your node subscribes to the chatter topic which is of type   s   std_msgs.msgs.String
    rospy.spin()
    # keeps your node from exiting until the node has been shutdown


if __name__ == '__main__':
    listener()
