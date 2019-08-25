## Manipulator

### Launch this in separate terminal tabs
Launch rviz simulation that will send angles to ros topic "/joint_states":
```bash
roslaunch my_arm_xacro demo.launch rviz_tutorial:=true
```
Important - change ttyUSB0 to your arduino port. Setup arduino connection:
```bash
sudo chmod -R 777 /dev/ttyUSB0
```
Send  data to arduino:
```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
```
