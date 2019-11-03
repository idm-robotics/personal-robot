#!/usr/bin/env python2

import rospy
import moveit_commander
from grasp_detection.msg import Grasp
from moveit_msgs.msg import moveit_msgs, geometry_msgs
import tf
import ik_solver_test
from geometry_msgs.msg import PointStamped

ik_solver = ik_solver_test.IKSolverTester()
tf_listener = tf.TransformListener()

# ('Goal target coordinates ', 0.027420001853080023, 0.7070000469684601, 0.010100000670978)


def callback(grasp):
    try:
        (trans, rot) = tf_listener.lookupTransform('/base_link', '/camera_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

    print('Transform: ', trans)
    print('Rotation: ', rot)

    left_point = grasp.left_point
    right_point = grasp.right_point
    x = (left_point.x + right_point.x) / 2
    y = (left_point.y + right_point.y) / 2
    z = (left_point.z + right_point.z) / 2

    point = PointStamped()
    point.point.x = x
    point.point.y = y
    point.point.z = z
    point.header.frame_id = 'camera_rgb_optical_frame'
    print(grasp.header)

    transformed_point = tf_listener.transformPoint('base_link', point)
    print("===========Point===============")
    print(point)
    print("===========TRANSFORMER===============")
    print(transformed_point)
    ik_solver.move_xyz(transformed_point.point.x, transformed_point.point.y, transformed_point.point.z)

    # x: -0.21489746767595763
    # y: 0.3179554308020345
    # z: -0.008753333914847445

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
