#!/usr/bin/env python

import message_filters
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from pointcloud_conversion import PointCloudConverter
from publisher import GraspPublisher
from grasp_detection.msg import Grasp
from visualization_msgs.msg import Marker

GRAB_INDEX = 0.25
x = 255
y = 265
x1 = 350
y1 = 360

marker_pub = rospy.Publisher('grasp_marker', Marker, queue_size=100)
grasp_pub = rospy.Publisher('grasp', Grasp, queue_size=100)


def get_sidepoints(left, top, right, bottom, grab):
    left_point = (int(left + (right - left) * grab), (top + bottom) // 2)
    right_point = (int(right - (right - left) * grab), (top + bottom) // 2)
    return left_point, right_point


def callback(rgb_image, depth_image, camera_info):
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
        image = bridge.imgmsg_to_cv2(rgb_image, "bgr8")

        cv_depth_image = bridge.imgmsg_to_cv2(depth_image)

        left_top_corner = (x, y)
        right_bottom_corner = (x1, y1)

        left_cup_point, right_cup_point = get_sidepoints(*left_top_corner, *right_bottom_corner, GRAB_INDEX)

        left_depth = cv_depth_image[left_cup_point]
        right_depth = cv_depth_image[right_cup_point]

        print('left_point:', left_depth)
        print('right_point:', right_depth)

        pc_converter = PointCloudConverter(camera_info)
        lp = pc_converter.convert(*left_cup_point, left_depth)
        rp = pc_converter.convert(*right_cup_point, right_depth)

        GraspPublisher.publish_marker(marker_pub, lp, rp)
        GraspPublisher.publish_grasp(grasp_pub, lp, rp)

        # draw_box(left_top_corner, right_bottom_corner)
        # draw_sidepoints(*left_top_corner, *right_bottom_corner, GRAB_INDEX)

    except CvBridgeError as e:
        print(e)

        # cv2.imwrite("image_try", image)
        # cv2.imshow('image', image)
        # cv2.waitKey(2)


def listener():
    rospy.init_node('listener', anonymous=True)
    rgb_sub = message_filters.Subscriber("camera/rgb/image_rect_color", Image)
    depth_sub = message_filters.Subscriber("camera/depth_registered/image", Image)
    camera_info_sub = message_filters.Subscriber("camera/depth_registered/camera_info", CameraInfo)

    ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, camera_info_sub], 1, 1)
    ts.registerCallback(callback)

    # declares that your node subscribes to the chatter topic which is of type   s   std_msgs.msgs.String
    rospy.spin()
    # keeps your node from exiting until the node has been shutdown


if __name__ == '__main__':
    listener()
