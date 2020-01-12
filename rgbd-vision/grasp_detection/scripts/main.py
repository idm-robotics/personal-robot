#!/usr/bin/env python

import message_filters
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from pointcloud_conversion import PointCloudConverter
from publisher import GraspPublisher
from grasp_detection.msg import Grasp
from object_detection.msg import DetectedObjectArray
from visualization_msgs.msg import Marker

GRAB_MARGIN = 0.25
BOTTOM_MARGIN = 0.25
DEPTH_BOX_INDENT = 3

marker_pub = rospy.Publisher('grasp_marker', Marker, queue_size=100)
grasp_pub = rospy.Publisher('grasp', Grasp, queue_size=100)
bridge = CvBridge()


def get_sidepoints(left, top, right, bottom, grab):
    indent = (right - left) * grab
    y = (top + bottom) // 2
    left_point = int(left + indent), y
    right_point = int(right - indent), y
    return left_point, right_point


def get_center_point(left, top, right, bottom):
    margin = (top - bottom) * BOTTOM_MARGIN
    x = (left + right) // 2
    return x, int(bottom + margin)


def get_depth(cv_depth_image, point):
    depth = cv_depth_image[point[::-1]]
    if np.isnan(depth):
        x_left_indent = point[0] - DEPTH_BOX_INDENT
        x_right_indent = point[0] + DEPTH_BOX_INDENT
        y_bottom_indent = point[1] - DEPTH_BOX_INDENT
        y_top_indent = point[1] + DEPTH_BOX_INDENT
        box = cv_depth_image[y_bottom_indent:y_top_indent, x_left_indent:x_right_indent]
        box_reshaped = np.reshape(np.array(box), -1)
        box_reshaped = box_reshaped[~np.isnan(box_reshaped)]
        return np.median(box_reshaped)
    return depth


# TODO: refactor this callback
def callback(rgb_image, depth_image, camera_info, object_detection_boxes: DetectedObjectArray):
    def draw_sidepoints(left_point, right_point):
        cv2.circle(image, left_point, 2, (0, 255, 0), 2)
        cv2.circle(image, right_point, 2, (0, 255, 0), 2)

    def draw_box(a, b):
        cv2.rectangle(image, a, b, (255, 0, 0), 2)

    image = None
    try:
        image = bridge.imgmsg_to_cv2(rgb_image, "bgr8")
        rospy.loginfo(image.shape)
        height, width, channel = image.shape

        cv_depth_image = bridge.imgmsg_to_cv2(depth_image)

        target_boxes = [box for box in object_detection_boxes.data if box.category == 'cup']
        if not target_boxes:
            rospy.loginfo('No cups were found')
            return
        box = target_boxes[0]
        rospy.loginfo(box)
        left_top_corner = (max(0, box.left), min(height, box.top))
        right_bottom_corner = (min(width, box.right), max(0, box.bottom))

        left_cup_point, right_cup_point = get_sidepoints(*left_top_corner, *right_bottom_corner, GRAB_MARGIN)
        center_cup_point = get_center_point(*left_top_corner, *right_bottom_corner)

        left_depth = get_depth(cv_depth_image, left_cup_point)
        right_depth = get_depth(cv_depth_image, right_cup_point)
        center_depth = get_depth(cv_depth_image, right_cup_point)

        if np.isnan(left_depth):
            rospy.logwarn(f'left depth is nan')
            if not np.isnan(right_depth):
                left_depth = right_depth
        if np.isnan(right_depth):
            rospy.logwarn(f'right depth is nan')
            if not np.isnan(left_depth):
                right_depth = left_depth

        rospy.loginfo(f'left_cup_point: {left_cup_point}')
        rospy.loginfo(f'right_cup_point: {right_cup_point}')
        rospy.loginfo(f'center_cup_point: {center_cup_point}')
        rospy.loginfo(f'left_depth: {left_depth}')
        rospy.loginfo(f'right_depth: {right_depth}')
        rospy.loginfo(f'center_depth: {center_depth}')

        pc_converter = PointCloudConverter(camera_info)
        left_point = pc_converter.convert(*left_cup_point, left_depth)
        right_point = pc_converter.convert(*right_cup_point, right_depth)
        center_point = pc_converter.convert(*center_cup_point, center_depth)

        GraspPublisher.publish_marker(marker_pub, left_point, right_point, center_point)
        GraspPublisher.publish_grasp(grasp_pub, left_point, right_point, center_point)

        draw_box(left_top_corner, right_bottom_corner)
        draw_sidepoints(left_cup_point, right_cup_point)

    except CvBridgeError as e:
        print(e)

    # cv2.imshow('image', image)
    # cv2.waitKey(2)


def listener():
    rospy.init_node('listener', anonymous=True)
    rgb_sub = message_filters.Subscriber("camera/rgb/image_rect_color", Image)
    depth_sub = message_filters.Subscriber("camera/depth_registered/image", Image)
    camera_info_sub = message_filters.Subscriber("camera/depth_registered/camera_info", CameraInfo)
    object_detection_sub = message_filters.Subscriber("object_detection/boxes", DetectedObjectArray)

    ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, camera_info_sub, object_detection_sub],
                                                     100, 20)
    ts.registerCallback(callback)

    # declares that your node subscribes to the chatter topic which is of type   s   std_msgs.msgs.String
    rospy.spin()
    # keeps your node from exiting until the node has been shutdown


if __name__ == '__main__':
    listener()
