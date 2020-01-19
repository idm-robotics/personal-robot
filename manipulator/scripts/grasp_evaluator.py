#!/usr/bin/env python2
import moveit_commander
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from grasp_detection.msg import Grasp as GraspDetection
from moveit_msgs.msg import Grasp as GraspObject
from moveit_msgs.msg import moveit_msgs
from trajectory_msgs.msg import JointTrajectory
from gripper.msgs.msg import GripperTranslation
from tf.transformations import quaternion_from_matrix
from factories import ObjectFactory, GraspFactory


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

    def print_current_pose(self):
        print('====== Current Pose ======')
        print(self.move_group.get_current_pose())
        print(self.move_group.get_current_rpy())

    def move_pose(self, pose):
        self.move_group.set_pose_target(pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    @staticmethod
    def point_to_pose(position, direction_vector, upper_vector):
        pose = PoseStamped()
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]

        rotation_matrix = np.eye(4)
        rotation_matrix[:3, 0] = direction_vector
        rotation_matrix[:3, 1] = upper_vector
        rotation_matrix[:3, 2] = np.cross(rotation_matrix[:3, 0], rotation_matrix[:3, 1])

        print(rotation_matrix)

        rotation_quaternion = quaternion_from_matrix(rotation_matrix)
        print("rotation_quaternion:", rotation_quaternion)

        pose.pose.orientation.x = rotation_quaternion[0]
        pose.pose.orientation.y = rotation_quaternion[1]
        pose.pose.orientation.z = rotation_quaternion[2]
        pose.pose.orientation.w = rotation_quaternion[3]

        pose.header.frame_id = 'camera_rgb_optical_frame'
        return pose

    def normalize(self, v):
        norm = np.linalg.norm(v)
        if norm == 0:
            return v
        return v / norm

    def calculate_indent_position(self, position, normal, coefficient):  # np.array
        normalized_normal = self.normalize(normal)
        return position + coefficient * normalized_normal


class BaseMovement:
    @staticmethod
    def __get_grasp_pose(grasp_coordinates):
        left_point = np.array([grasp_coordinates.left_point.x,
                               grasp_coordinates.left_point.y,
                               grasp_coordinates.left_point.z])
        right_point = np.array([grasp_coordinates.right_point.x,
                                grasp_coordinates.right_point.y,
                                grasp_coordinates.right_point.z])
        center_point = np.array([grasp_coordinates.center_point.x,
                                 grasp_coordinates.center_point.y,
                                 grasp_coordinates.center_point.z])
        object_position = (left_point + right_point) / 2

        cup_normal_vector = np.cross(left_point - center_point, right_point - center_point)
        cup_right_vector = right_point - left_point

        gripper_position = IKSolver.calculate_indent_position(object_position, cup_normal_vector, 0.1)

        print("Target position: ", object_position)
        print("Final position: ", gripper_position)

        pose = IKSolver.point_to_pose(gripper_position, cup_right_vector, cup_normal_vector)

        return pose

    @staticmethod
    def __get_transform_pose(pose):
        transformed_pose = tf.TransformListener.transformPose('base_link', pose)
        print("===========Pose===============")
        print(pose)
        print("===========TRANSFORMER===============")
        print(transformed_pose)

        return transformed_pose

    # def __move_to_grasping_pose(self, grasp_coordinates):
    #     pose = self.__get_grasp_pose(grasp_coordinates)
    #     transformed_pose = __get_transform_pose(pose)
    #     IKSolver.move_pose(transformed_pose)
    #
    #     print('Goal target pose coordinates ', pose)
    #     IKSolver.print_current_pose()
    #
    # def pick(self, grasp_coordinates):
    #     self.__move_to_grasping_pose(grasp_coordinates)

    # def __move_to_release_pose(self):
    #     # normalize, calculate_indent
    #     self.__move_to_pose()
    #
    # def __release(self):
    #     self.__holding = False
    #     pass
    #
    # def place(self):
    #     if self.__holding:
    #         self.__move_to_release_pose(point)
    #         self.__release()



class GraspEvaluator:
    def __init__(self, user_task):
        self.ik_solver = IKSolver()
        self.tf_listener = tf.TransformListener()

    def do_request(self):
        # will I spawn a box every spin?
        object = ObjectFactory.build()
        grasp = GraspFactory.build(grasp_coordinates)
        self.ik_solver.move_group.pick('object', grasp)
        # attaching object is missing from the scheme
        # TODO: add gripper behaviour
        # TODO: add gripper and end_eddector

    def callback(self, grasp):
        self.do_request()

    def listener(self):
        # rospy.init_node('grasp_evaluator', anonymous=True)
        rospy.Subscriber('grasp_coordinates', GraspDetection, self.callback, queue_size=1)
        rospy.spin()


if __name__ == '__main__':
    GraspEvaluator('pick').listener()
