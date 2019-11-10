#!/usr/bin/env python2
import moveit_commander
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from grasp_detection.msg import Grasp as GraspDetection
from moveit_msgs.msg import moveit_msgs
from tf.transformations import quaternion_from_euler


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


class GraspEvaluator:
    def __init__(self):
        self.ik_solver = IKSolver()
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
