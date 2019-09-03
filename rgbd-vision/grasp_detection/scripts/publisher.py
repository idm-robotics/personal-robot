import rospy
from geometry_msgs.msg import Point
from grasp_detection.msg import Grasp
from rospy import Publisher
from visualization_msgs.msg import Marker


class GraspPublisher:
    @staticmethod
    def publish_grasp(pub: Publisher, left_point: Point, right_point: Point):
        grasp = Grasp()
        grasp.left_point = left_point
        grasp.right_point = right_point
        pub.publish(grasp)

    @staticmethod
    def publish_marker(pub: Publisher, left_point: Point, right_point: Point):
        # TODO: investigate, why we need this transformation for markers
        def to_marker_point(point: Point):
            return Point(point.z, point.x, -point.y)

        marker = Marker()
        marker.header.frame_id = "camera_link"

        marker.type = marker.POINTS
        marker.action = marker.ADD

        marker.points = [to_marker_point(left_point), to_marker_point(right_point)]
        t = rospy.Duration()
        marker.lifetime = t
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0

        pub.publish(marker)
