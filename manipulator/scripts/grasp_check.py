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
print "============ Planning frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print "============ End effector link: %s" % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print "============ Available Planning Groups:", robot.get_group_names()

def setup_joint_config():
    joint_goal = group.get_current_joint_values()
    print(joint_goal)
    joint_goal[0] = pi/8
    # joint_goal[1] = pi/4
    # joint_goal[2] = -pi/2
    # joint_goal[3] = pi/3
    # joint_goal[4] = 0
    # joint_goal[5] = pi / 3

    group.go(joint_goal, wait=True)
    group.stop()
    print(group.get_current_pose())

# setup_joint_config()

def move_group(x=0, y=0, z=0, w=1):
    """ Moves hand to the provided position. You take some initial orientation,
     and then rotate it around vector (x,y,z) by acos(w).
    """

    x= -0.0429069797522
    y= -0.227064386365
    z= 0.0660294390438
    orientation_x= -0.39279870611
    orientation_y= 0.588049564506
    orientation_z= 0.392836679399
    orientation_w= 0.587865826088
    orientation_w=1

    pose_goal = geometry_msgs.msg.Pose()
    group.get_current_pose()
    print('i am here')
    # pose_goal.orientation.x = orientation_x
    # pose_goal.orientation.y = orientation_y
    # pose_goal.orientation.z = orientation_z
    pose_goal.orientation.w = orientation_w
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    print(pose_goal)
    group.set_pose_target(pose_goal)
    group.set_planning_time(20)
    print('before go')
    plan = group.go(wait=True)
    print('after go')
    group.stop()
    group.clear_pose_targets()
    print('NOW trajectory')



if __name__ == '__main__':
    for i in range(5):
        move_group()
        time.sleep(1)
