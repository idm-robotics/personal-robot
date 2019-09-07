## Manipulator

### Launch this in separate terminal tabs
Launch rviz simulation that will send angles to ros topic "/joint_states":
```bash
roslaunch manipulator demo.launch
```
After launching demo.launch run grasp_evaluator.py from manipulator node
```bash
rosrun manipulator grasp_evaluator.py
```
Important - change ttyUSB0 to your arduino port. Setup arduino connection:
```bash
sudo chmod -R 777 /dev/ttyUSB0
```
Send  data to arduino:
```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
```
