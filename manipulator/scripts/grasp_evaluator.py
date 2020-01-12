#!/usr/bin/env python2
import moveit_commander
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from grasp_detection.msg import Grasp as GraspDetection
from moveit_msgs.msg import moveit_msgs
from tf.transformations import quaternion_from_matrix


class IKSolver:
    def __init__(self):
        moveit_commander.roscpp_initialize(args=[])
        rospy.init_node('ik_solver', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
        self.print_base_info()

    def print_base_info(self):
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        group_names = self.robot.get_group_names()
        print("============ Available Planning Groups: %s" % group_names)

    def get_current_quaternion(self):
        q = self.move_group.get_current_pose().pose.orientation
        return np.array([q.x, q.y, q.z, q.w])

    def print_current_pose(self):
        print('====== Current Pose ======')
        print(self.move_group.get_current_pose())
        print(self.move_group.get_current_rpy())

    def move_xyz(self, x=0.0, y=0.0, z=0.0):
        self.move_group.set_position_target([x, y, z])
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def move_pose(self, pose):
        self.move_group.set_pose_target(pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    @staticmethod
    def point_to_pose(position, direction_vector, upper_vector):
        point = PoseStamped()
        point.pose.position.x = position[0]
        point.pose.position.y = position[1]
        point.pose.position.z = position[2]

        rotation_matrix = np.eye(4)
        rotation_matrix[:3, 0] = direction_vector
        rotation_matrix[:3, 1] = upper_vector
        rotation_matrix[:3, 2] = np.cross(rotation_matrix[:3, 0], rotation_matrix[:3, 1])

        print(rotation_matrix)

        rotation_quaternion = quaternion_from_matrix(rotation_matrix)
        print("rotation_quaternion:", rotation_quaternion)

        point.pose.orientation.x = rotation_quaternion[0]
        point.pose.orientation.y = rotation_quaternion[1]
        point.pose.orientation.z = rotation_quaternion[2]
        point.pose.orientation.w = rotation_quaternion[3]

        point.header.frame_id = 'camera_rgb_optical_frame'
        return point


class GraspEvaluator:
    def __init__(self):
        self.ik_solver = IKSolver()
        self.tf_listener = tf.TransformListener()

    def callback(self, grasp):
        left_point = np.array([grasp.left_point.x, grasp.left_point.y, grasp.left_point.z])
        right_point = np.array([grasp.right_point.x, grasp.right_point.y, grasp.right_point.z])
        center_point = np.array([grasp.center_point.x, grasp.center_point.y, grasp.center_point.z])
        target_position = (left_point + right_point) / 2

        cup_normal_vector = np.cross(left_point - center_point, right_point - center_point)
        cup_right_vector = right_point - left_point

        point = self.ik_solver.point_to_pose(target_position, cup_right_vector, cup_normal_vector)

        transformed_point = self.tf_listener.transformPose('base_link', point)
        print("===========Point===============")
        print(point)
        print("===========TRANSFORMER===============")
        print(transformed_point)
        # self.ik_solver.move_xyz(transformed_point.point.x, transformed_point.point.y, transformed_point.point.z)
        self.ik_solver.move_pose(transformed_point)

        print('Goal target coordinates ', point)
        self.ik_solver.print_current_pose()

    def listener(self):
        # rospy.init_node('grasp_evaluator', anonymous=True)
        rospy.Subscriber('grasp', GraspDetection, self.callback, queue_size=1)
        rospy.spin()


if __name__ == '__main__':
    GraspEvaluator().listener()
