#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import CameraInfo
from pointcloud_conversion import PointCloudConverter
# from python_subscriber_node import get_sidepoints

# rate = rospy.Rate(5)


def publish_marker(pub, lp, rp):
    left_point = Point()
    right_point = Point()

    left_point.x = lp[0]
    left_point.y = lp[1]
    left_point.z = lp[2]

    right_point.x = rp[0]
    right_point.y = rp[1]
    right_point.z = rp[2]

    marker = Marker()
    marker.header.frame_id = "camera_link"

    marker.type = marker.POINTS
    marker.action = marker.ADD
    # marker.pose.orientation.w = 1
    marker.points = [left_point, right_point]
    t = rospy.Duration()
    marker.lifetime = t
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 1.0

    pub.publish(marker)
    # rate.sleep()


# def publisher():
#     GRAB_INDEX = 0.15
#     x = 255
#     y = 265
#     x1 = 350
#     y1 = 360
#     left_top_corner = (x, y)
#     right_bottom_corner = (x1, y1)
#
#     cup_points = get_sidepoints(*left_top_corner,
#                                 *right_bottom_corner, GRAB_INDEX)
#     to_pointcloud = PointCloudConverter(CameraInfo)
#     lp = to_pointcloud.convert(cup_points[0])
#     rp = to_pointcloud.convert(cup_points[1])
#
#     rate = rospy.Rate(5)
#
#     left_point = Point()
#     right_point = Point()
#
#     left_point.x = lp[0]
#     left_point.y = lp[1]
#     left_point.z = lp[2]
#
#     right_point.x = rp[0]
#     right_point.y = rp[1]
#     right_point.z = rp[2]
#
#     while not rospy.is_shutdown():
#         pub = rospy.Publisher('Publish_Ilia', Marker, queue_size=100)
#         marker = Marker()
#         marker.header.frame_id = "camera_link"
#
#         marker.type = marker.POINTS
#         marker.action = marker.ADD
#         marker.points = [left_point, right_point]
#         t = rospy.Duration()
#         marker.lifetime = t
#         marker.scale.x = 0.1
#         marker.scale.y = 0.1
#         marker.scale.z = 0.1
#         marker.color.a = 1.0
#         marker.color.r = 1.0
#
#         pub.publish(marker)
#         rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('publish_points', anonymous=True)
        # publisher()
    except rospy.ROSInterruptException:
        print('something is wrong')
        pass
