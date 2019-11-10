#!/usr/bin/env python2
import numpy as np
import rospy
from grasp_detection.msg import Grasp as GraspDetection
import tf
import ik_solver_test
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from tf.transformations import quaternion_from_euler


class GraspEvaluator:
    def __init__(self):
        self.ik_solver = ik_solver_test.IKSolverTester()
        self.tf_listener = tf.TransformListener()

    @staticmethod
    def cartesian_to_spherical(x, y, z):
        xy = x ** 2 + y ** 2
        spherical_x = np.sqrt(xy + z ** 2)
        spherical_y = np.arctan2(np.sqrt(xy), z)  # for elevation angle defined from Z-axis down
        # ptsnew[:,4] = np.arctan2(xyz[:,2], np.sqrt(xy)) # for elevation angle defined from XY-plane up
        spherical_z = np.arctan2(y, x)
        return spherical_x, spherical_y, spherical_z

    def callback(self, grasp):
        left_point = grasp.left_point
        right_point = grasp.right_point
        x = (left_point.x + right_point.x) / 2
        y = (left_point.y + right_point.y) / 2
        z = (left_point.z + right_point.z) / 2

        # point = PointStamped()
        # point.point.x = x
        # point.point.y = y
        # point.point.z = z
        point = PoseStamped()
        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.position.z = z

        roll, pitch, yaw = self.cartesian_to_spherical(x, y, z)
        print("roll, pitch, yaw: ", roll, pitch, yaw)
        q_x, q_y, q_z, q_w = quaternion_from_euler(roll, pitch, yaw)
        print(q_x, q_y, q_z, q_w)

        point.pose.orientation.x = q_x
        point.pose.orientation.y = q_y
        point.pose.orientation.z = q_z
        point.pose.orientation.w = q_w

        point.header.frame_id = 'camera_rgb_optical_frame'
        print(grasp.header)

        # transformed_point = self.tf_listener.transformPoint('base_link', point)
        transformed_point = self.tf_listener.transformPose('base_link', point)
        print("===========Point===============")
        print(point)
        print("===========TRANSFORMER===============")
        print(transformed_point)
        # self.ik_solver.move_xyz(transformed_point.point.x, transformed_point.point.y, transformed_point.point.z)
        self.ik_solver.move_pose(transformed_point)

        print('Goal target coordinates ', x, y, z)
        self.ik_solver.print_current_pose()

    # def create_grasp(self, x, y, z):
    #     grasp = Grasp()
    #     grasp.pre_grasp_approach.direction.vector.z = 1
    #     grasp.pre_grasp_approach.direction.header.frame_id = 'endeff'
    #     grasp.pre_grasp_approach.min_distance = 0.04
    #     grasp.pre_grasp_approach.desired_distance = 0.10
    #
    #     point = PoseStamped()
    #     point.pose.position.x = x
    #     point.pose.position.y = y
    #     point.pose.position.z = z
    #     point.
    #     point.header.frame_id = 'camera_rgb_optical_frame'
    #     grasp.grasp_pose.pose =


    def listener(self):
        # rospy.init_node('grasp_evaluator', anonymous=True)
        rospy.Subscriber('grasp', GraspDetection, self.callback, queue_size=1)
        rospy.spin()


if __name__ == '__main__':
    GraspEvaluator().listener()
