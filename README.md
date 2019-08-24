## Personal Robot Project

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
Here is the prototype version of arduino code:
https://gist.githubusercontent.com/dgrachev28/d8fa42e6630358929453ae1be7a31c1a/raw/74ab6a59d83b0c44d5ba0686e76ca6dfa06b6c01/gistfile1.txt

This is refactored code with implemented class for a joint:
https://gist.githubusercontent.com/ExMiracle/65e49ace7343694eb7de1e7af042bbaa/raw/75f812391587d4065932c5b901c619cb4f203502/gist

### TODO: Denis, upload our urdf and meshes files to the repository. If possible, upload whole robot folder.