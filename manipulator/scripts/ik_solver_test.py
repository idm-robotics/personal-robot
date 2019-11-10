#!/usr/bin/env python2

import sys

import moveit_commander
import rospy
from moveit_msgs.msg import moveit_msgs


class IKSolverTester:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ik_solver_test', anonymous=True)

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


def main():
    tester = IKSolverTester()
    for i in range(5):
        tester.print_current_pose()
        x = float(input("x = "))
        y = float(input("y = "))
        z = float(input("z = "))
        print("Got values:", x, y, z)
        tester.move_xyz(x, y, z)


if __name__ == '__main__':
    main()
