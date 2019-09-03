#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_msgs.msg import grasp

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "hand"
group = moveit_commander.MoveGroupCommander(group_name)

def setup_joint_config():
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    group.go(joint_goal, wait=True)
    group.stop()

setup_joint_config()

def callback(grasp: Grasp):
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
    def move_group(x, y, z, w=1):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = w
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        group.set_pose_target(pose_goal)

        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory);
# TODO: PointCloud has to have complete integration in the world so collision matrix will detect collisions with objects
    
    

def listener():
    rospy.init_node('grasp_evaluator', anonymous=True)
    subscriber = rospy.Subscriber('grasp', Grasp, callback, queue_size=1)

    rospy.spin


if __name__ == '__main__':
    listener()
