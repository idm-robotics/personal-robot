#!/usr/bin/env python2

import rospy
import moveit_commander
from grasp_detection.msg import Grasp
from moveit_msgs.msg import moveit_msgs, geometry_msgs
from math import pi

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)

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
    joint_goal[0] = pi
    joint_goal[1] = pi/4
    joint_goal[2] = -pi/2
    joint_goal[3] = pi/3
    # joint_goal[4] = 0
    # joint_goal[5] = pi / 3

    group.go(joint_goal, wait=True)
    group.stop()


# setup_joint_config()


def callback(grasp):
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    # setup_joint_config()
    # joint_goal = group.get_current_joint_values()
    # print(joint_goal)
    # print(group.get_current_pose())
    def move_group(x, y, z, w=1):
        """ Moves hand to the provided position. You take some initial orientation,
         and then rotate it around vector (x,y,z) by acos(w).
        """
        x = -0.000491245989392
        y = 0.182776639784
        z = 0.0511645172446
        orientation_x = 0.561020917974
        orientation_y = 0.560933299535
        orientation_z = -0.430454621727
        orientation_w = 0.430485983166


        pose_goal = geometry_msgs.msg.Pose()
        group.get_current_pose()
        pose_goal.orientation.x = orientation_x
        pose_goal.orientation.y = orientation_y
        pose_goal.orientation.z = orientation_z
        pose_goal.orientation.w = orientation_w
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal = group.get_current_pose()
  #   x: -0.000491245989392
  #   y: 0.182776639784
  #   z: 0.0511645172446
  # orientation: 
  #   x: 0.561020917974
  #   y: 0.560933299535
  #   z: -0.430454621727
  #   w: 0.430485983166


        print(pose_goal)
        group.set_pose_target(pose_goal)
        print('before go')
        plan = group.go(wait=True)
        print('after go')
        group.stop()
        group.clear_pose_targets()
        print('NOW trajectory')
        # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        # display_trajectory.trajectory_start = robot.get_current_state()
        # print (robot.get_current_state)
        # display_trajectory.trajectory.append(plan)
        # display_trajectory_publisher.publish(display_trajectory)
        # print("CONRANTS")
        # group.execute(plan, wait=True) 
        # print('executed plan')
        # # TODO: PointCloud has to have complete integration in the world
        # so collision matrix will detect collisions with objects

    left_point = grasp.left_point
    right_point = grasp.right_point
    x = (left_point.x + right_point.x) / 2
    y = (left_point.y + right_point.y) / 2
    z = (left_point.z + right_point.z) / 2
    move_group(x, y, z)

def listener():
    rospy.init_node('grasp_evaluator', anonymous=True)
    subscriber = rospy.Subscriber('grasp', Grasp, callback, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    listener()
