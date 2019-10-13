#!/usr/bin/env python2

import rospy
import moveit_commander
from grasp_detection.msg import Grasp
from moveit_msgs.msg import moveit_msgs, geometry_msgs
from math import pi
import ik_solver_test


# def setup_joint_config():
#     joint_goal = group.get_current_joint_values()
#     joint_goal[0] = 0
#     joint_goal[1] = -pi / 4
#     joint_goal[2] = 0
#     joint_goal[3] = -pi / 2
#     joint_goal[4] = 0
#     joint_goal[5] = pi / 3

#     group.go(joint_goal, wait=True)
#     group.stop()


# setup_joint_config()


# robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()
# group_name = "hand"
# group = moveit_commander.MoveGroupCommander(group_name)

ik_solver = ik_solver_test.IKSolverTester()

# ('Goal target coordinates ', 0.027420001853080023, 0.7070000469684601, 0.010100000670978)

def callback(grasp):

    left_point = grasp.left_point
    right_point = grasp.right_point
    x = (left_point.x + right_point.x) / 2
    y = (left_point.y + right_point.y) / 2
    z = (left_point.z + right_point.z) / 2
    ik_solver.move_xyz(x, y, z)

    print('Goal target coordinates ', x, y, z)
    ik_solver.print_current_pose()

    # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
    #                                                moveit_msgs.msg.DisplayTrajectory,
    #                                                queue_size=20)
    #
    # def move_group(x, y, z, w=1):
    #     """ Moves hand to the provided position. You take some initial orientation,
    #      and then rotate it around vector (x,y,z) by acos(w).
    #     """
    #     pose_goal = geometry_msgs.msg.Pose()
    #     pose_goal.orientation.w = w
    #     pose_goal.position.x = x
    #     pose_goal.position.y = y
    #     pose_goal.position.z = z
    #     group.set_pose_target(pose_goal)
    #
    #     plan = group.go(wait=True)
    #     group.stop()
    #     group.clear_pose_targets()
    #
    #     display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    #     display_trajectory.trajectory_start = robot.get_current_state()
    #     display_trajectory.trajectory.append(plan)
    #     display_trajectory_publisher.publish(display_trajectory)
    #
    #     # TODO: PointCloud has to have complete integration in the world
    #     # so collision matrix will detect collisions with objects
    #
    # left_point = grasp.left_point
    # right_point = grasp.right_point
    # x = (left_point.x + right_point.x) // 2
    # y = (left_point.y + right_point.y) // 2
    # z = (left_point.z + right_point.z) // 2
    # move_group(x, y, z)

def listener():
    # rospy.init_node('grasp_evaluator', anonymous=True)
    subscriber = rospy.Subscriber('grasp', Grasp, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    listener()
