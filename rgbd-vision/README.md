## RGBD Vision

### Quick start
[Start guide](quick-start.md)

### Overview
The main goal is described in this article https://arxiv.org/pdf/1903.06684.pdf

Manipulation pipeline consists of several steps:
1. Instance segmentation
2. 3D keypoints detection
3. Robot action planning
4. Geometric grasping
5. Action execution

This package contains steps 1, 2 and 4.

### Hardware
We use Xbox 360 Kinect as RGBD camera

### Software
ROS https://www.ros.org/ is used in our desktop ubuntu 16.04 or 18.04

To connect Kinect to PC use openni_launch http://wiki.ros.org/openni_launch
