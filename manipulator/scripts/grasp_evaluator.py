#!/usr/bin/env python

import rospy


def callback(grasp: Grasp):
    pass


def listener():
    rospy.init_node('grasp_evaluator', anonymous=True)
    subscriber = rospy.Subscriber('grasp', Grasp, callback, queue_size=1)

    rospy.spin


if __name__ == '__main__':
    listener()
