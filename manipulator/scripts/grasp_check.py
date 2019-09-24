#!/usr/bin/env python2

import rospy
import moveit_commander
from grasp_detection.msg import Grasp
from moveit_msgs.msg import moveit_msgs, geometry_msgs
from math import pi
import sys
import time

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('grasp_evaluator', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups: %s" % group_names)


def setup_joint_config():
    joint_goal = group.get_current_joint_values()
    print(joint_goal)
    joint_goal[0] = pi / 8
    # joint_goal[1] = pi/4
    # joint_goal[2] = -pi/2
    # joint_goal[3] = pi/3
    # joint_goal[4] = 0
    # joint_goal[5] = pi / 3

    group.go(joint_goal, wait=True)
    group.stop()
    print(group.get_current_pose())

def move_group(x=0, y=0, z=0, w=1):
    """ Moves hand to the provided position. You take some initial orientation,
     and then rotate it around vector (x,y,z) by acos(w).
    """
    # x = -0.0529069797522
    # y = -0.257064386365
    # z = 0.3660294390438

    x = -0.0311485834424
    y = 0.153716836359
    z = 0.455814740946

    orientation_x = -0.645015871433
    orientation_y = -0.00841119708656
    orientation_z = 0.0623725862557
    orientation_w = 0.76157300231

# x = 0.162675875772
    # y = 0.16987468389
    # z = -0.106007194841

    # orientation_x = -0.39279870611
    # orientation_y = 0.588049564506
    # orientation_z = 0.392836679399
    # orientation_w = 0.587865826088
    # orientation_w=1

    # x = 0.0299718021659
    # y = -0.604998081063
    # z = 0.1319873878
    # orientation_x = 3.53519987677e-05
    # orientation_y = 0.70717619546
    # orientation_z = -3.53455566562e-05
    # orientation_w = 0.707037358331

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = orientation_x
    pose_goal.orientation.y = orientation_y
    pose_goal.orientation.z = orientation_z
    pose_goal.orientation.w = orientation_w
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    print('===current pose===')
    print(group.get_current_pose())
    print('===goal pose===')
    print(pose_goal)
    group.set_planning_time(20)
    group.set_joint_value_target(pose_goal, True)
    # group.set_pose_target(pose_goal)
    print('before go')
    plan = group.go(wait=True)
    print('after go')
    group.stop()
    group.clear_pose_targets()


if __name__ == '__main__':
    for i in range(5):
        move_group()
        time.sleep(1)
