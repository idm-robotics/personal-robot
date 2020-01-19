import rospy
from geometry_msgs.msg import Point
from grasp_detection.msg import Grasp
from rospy import Publisher
from visualization_msgs.msg import Marker


class GraspPublisher:
    @staticmethod
    def publish_grasp(pub: Publisher, left_point: Point, right_point: Point, center_point: Point):
        grasp_coordinates = Grasp()
        grasp_coordinates.left_point = left_point
        grasp_coordinates.right_point = right_point
        grasp_coordinates.center_point = center_point
        pub.publish(grasp_coordinates)

    @staticmethod
    def publish_marker(pub: Publisher, left_point: Point, right_point: Point, center_point: Point):
        def to_marker_point(point: Point):
            return Point(point.x, point.y, point.z)

        marker = Marker()
        marker.header.frame_id = "camera_rgb_optical_frame"

        marker.type = marker.POINTS
        marker.action = marker.ADD

        marker.points = [to_marker_point(left_point), to_marker_point(right_point), to_marker_point(center_point)]
        t = rospy.Duration()
        marker.lifetime = t
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0

        pub.publish(marker)
