## Manipulator

### Launch sequence for rviz simulation
From project folder build package to catkin workspace
```bash
./build.sh
```
Turn on depth registration for kinect.
```bash
roslaunch openni_launch openni.launch depth_registration:=true
```
In separate tab execute this code. This launch file does several things.
Firstly it launches rviz with the robot and then it executes several scripts
 from rgbd-vision and manipulator packages.
```bash
roslaunch manipulator manipulator.launch
```

Important - change ttyUSB0 to your arduino port. Setup arduino connection:
```bash
sudo chmod -R 777 /dev/ttyUSB0
```
Send  data to arduino for angle transformations:
```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
```

### Launch sequence details
#### In separate tabs open next commands:
```bash
roslaunch manipulator_moveit_config demo.launch
rosrun manipulator grasp_evaluator.py
rosrun object_detection main.py
rosrun object_detection main.py
```
